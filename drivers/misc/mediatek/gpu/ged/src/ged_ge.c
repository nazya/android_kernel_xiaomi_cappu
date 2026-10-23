/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "ged_ge.h"

#include <ged_bridge.h>

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/anon_inodes.h>

#include <ged_debugFS.h>

typedef struct {
	uint64_t unique_id;

	uint32_t region_num;
	void *data;
	uint32_t *region_sizes; /* sub data */
	uint32_t **region_data; /* sub data */

	struct list_head ge_entry_list;
} GEEntry;

#define GE_PERR(fmt, ...)   pr_err("[GRALLOC_EXTRA,%s:%d]" fmt, __FILE__, __LINE__, ##__VA_ARGS__)

static struct kmem_cache *gPoolCache;
static struct dentry *gDFSEntry;
static LIST_HEAD(ge_entry_list_head);
static DEFINE_MUTEX(ge_entry_list_lock);

/* region alloc and free lock */
static DEFINE_SPINLOCK(ge_raf_lock);

static uint64_t gen_unique_id(void)
{
	/* A collision may occur after 2^44 sec, aka 557,474 years,
	 * if we do allocate and free in 2^20 times per second.
	 */
	static uint64_t serial;

	unsigned long flags;
	uint64_t ret;

	spin_lock_irqsave(&ge_raf_lock, flags);
	ret = serial++;
	spin_unlock_irqrestore(&ge_raf_lock, flags);

	return ret;
}

static void *_ge_debugfs_seq_start(struct seq_file *m, loff_t *pos)
{
	mutex_lock(&ge_entry_list_lock);
	return seq_list_start_head(&ge_entry_list_head, *pos);
}
void _ge_debugfs_seq_stop(struct seq_file *m, void *v)
{
	mutex_unlock(&ge_entry_list_lock);
}
void *_ge_debugfs_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
	return seq_list_next(v, &ge_entry_list_head, pos);
}
int _ge_debugfs_seq_show(struct seq_file *m, void *v)
{
	const GEEntry *entry;
	size_t memory_size;
	size_t memory_ksize;
	unsigned long flags;
	uint32_t i;

	if (v == &ge_entry_list_head) {
		struct list_head *node;
		size_t entries = 0;

		list_for_each(node, &ge_entry_list_head)
			entries++;
		seq_puts(m, "================================================\n");
		seq_printf(m, "Total entries: %zu\n", entries);
		return 0;
	}

	entry = list_entry(v, GEEntry, ge_entry_list);
	memory_size = (sizeof(uint32_t) + sizeof(uint32_t *)) *
			entry->region_num;
	memory_ksize = ksize(entry->data);
	for (i = 0; i < entry->region_num; ++i) {
		void *region_data;

		spin_lock_irqsave(&ge_raf_lock, flags);
		region_data = entry->region_data[i];
		spin_unlock_irqrestore(&ge_raf_lock, flags);
		if (region_data) {
			memory_size += entry->region_sizes[i];
			memory_ksize += ksize(region_data);
		}
	}
	seq_printf(m, "GEEntry id:0x%llx memory size: %zu bytes, ksize: %zu bytes\n",
			entry->unique_id, memory_size, memory_ksize);
	return 0;
}
static const struct seq_operations gDEFEntryOps = {
	.start = _ge_debugfs_seq_start,
	.stop = _ge_debugfs_seq_stop,
	.next = _ge_debugfs_seq_next,
	.show = _ge_debugfs_seq_show,
};
static ssize_t _ge_debugfs_write_entry(const char __user *pszBuffer, size_t uiCount, loff_t uiPosition, void *pvData)
{
	return uiCount;
}

static int ge_entry_release(struct inode *inode, struct file *file)
{
	uint32_t i;
	GEEntry *entry = file->private_data;

	mutex_lock(&ge_entry_list_lock);
	list_del_init(&entry->ge_entry_list);
	mutex_unlock(&ge_entry_list_lock);

	/* remove object */
	for (i = 0; i < entry->region_num; ++i)
		kfree(entry->region_data[i]);
	kfree(entry->data);
	kmem_cache_free(gPoolCache, entry);
	return 0;
}

static const struct file_operations GEEntry_fops = {
	.owner = THIS_MODULE,
	.release = ge_entry_release,
};

int ged_ge_init(void)
{
	int flags = 0;

	gPoolCache = kmem_cache_create("gralloc_extra", sizeof(GEEntry), 0, flags, NULL);

	ged_debugFS_create_entry(
			"ge",
			NULL,
			&gDEFEntryOps,
			_ge_debugfs_write_entry,
			NULL,
			&gDFSEntry);

	return 0;
}

int ged_ge_exit(void)
{
	ged_debugFS_remove_entry(gDFSEntry);

	/* TODO : free all memory */
	kmem_cache_destroy(gPoolCache);

	return 0;
}

int ged_ge_alloc_prepare(uint32_t region_num, uint32_t *region_sizes,
		struct file **file)
{
	struct file *entry_file;
	int alloc_fd;
	uint32_t i;
	GEEntry *entry = (GEEntry *)kmem_cache_zalloc(gPoolCache, GFP_KERNEL);

	*file = NULL;

	if (!entry) {
		GE_PERR("alloc entry fail, size:%zu\n", sizeof(GEEntry));
		goto err_entry;
	}
	INIT_LIST_HEAD(&entry->ge_entry_list);

	alloc_fd = get_unused_fd_flags(O_CLOEXEC);

	if (alloc_fd < 0) {
		GE_PERR("get_unused_fd_flags() return %d\n", alloc_fd);
		goto err_fd;
	}

	entry->region_num = region_num;
	entry->data = kcalloc(region_num,
			sizeof(uint32_t *) + sizeof(uint32_t), GFP_KERNEL);
	if (!entry->data) {
		GE_PERR("alloc data fail, region count:%u\n", region_num);
		goto err_kmalloc;
	}

	/* Keep the pointer array naturally aligned on both 32- and 64-bit ABIs. */
	entry->region_data = entry->data;
	entry->region_sizes = (uint32_t *)(entry->region_data + region_num);
	for (i = 0; i < region_num; ++i)
		entry->region_sizes[i] = region_sizes[i];

	entry->unique_id = gen_unique_id();
	entry_file = anon_inode_getfile("gralloc_extra", &GEEntry_fops,
			entry, 0);
	if (IS_ERR(entry_file)) {
		GE_PERR("anon_inode_getfile() fail\n");
		goto err_entry_file;
	}

	*file = entry_file;
	return alloc_fd;

err_entry_file:
	kfree(entry->data);
err_kmalloc:
	put_unused_fd(alloc_fd);
err_fd:
	kmem_cache_free(gPoolCache, entry);
err_entry:
	return -1;
}

void ged_ge_alloc_commit(int ge_fd, struct file *file)
{
	GEEntry *entry = file->private_data;

	mutex_lock(&ge_entry_list_lock);
	list_add_tail(&entry->ge_entry_list, &ge_entry_list_head);
	mutex_unlock(&ge_entry_list_lock);
	fd_install(ge_fd, file);
}

void ged_ge_alloc_abort(int ge_fd, struct file *file)
{
	put_unused_fd(ge_fd);
	fput(file);
}

int ged_ge_alloc(uint32_t region_num, uint32_t *region_sizes)
{
	struct file *file;
	int ge_fd;

	ge_fd = ged_ge_alloc_prepare(region_num, region_sizes, &file);
	if (ge_fd >= 0)
		ged_ge_alloc_commit(ge_fd, file);
	return ge_fd;
}

static void dump_ge_regions(GEEntry *entry)
{
	uint32_t i;

	for (i = 0; i < entry->region_num; ++i)
		GE_PERR("ged_ge dump, %u/%u, s %u\n", i, entry->region_num, entry->region_sizes[i]);
}

/*
 * Region sizes are byte counts, while offsets and lengths are counts of
 * uint32_t words.  None can be negative, so keep their bridge and kernel
 * representations unsigned.  This preserves the four-byte field layout used
 * by prebuilt clients; a legacy negative bit pattern becomes a large value
 * and fails the bounds check below.
 *
 * The old (offset + size) * sizeof(uint32_t) expression could overflow before
 * it was compared with the region size.  Convert the byte capacity to complete
 * words instead, validate the offset, and compare the length with the remaining
 * words.  Short-circuit evaluation makes the subtraction safe, a trailing
 * partial word remains inaccessible, and constant division by four compiles to
 * a shift rather than a hardware divide.
 */
static int valid_parameters(GEEntry *entry, uint32_t region_id,
		uint32_t u32_offset, uint32_t u32_size)
{
	if (region_id < entry->region_num) {
		uint32_t region_words = entry->region_sizes[region_id] /
				sizeof(uint32_t);

		if (u32_offset <= region_words &&
			u32_size <= region_words - u32_offset)
			return 0;
	}

	GE_PERR("fail, invalid r_id %u, o %u, s %u\n",
			region_id, u32_offset, u32_size);

	dump_ge_regions(entry);

	return -EFAULT;
}

int ged_ge_get(int ge_fd, uint32_t region_id, uint32_t u32_offset,
		uint32_t u32_size, uint32_t *output_data)
{
	unsigned long flags;
	int err = 0;
	uint32_t i;
	GEEntry *entry = NULL;
	uint32_t *pregion_data = NULL;
	struct file *file = fget(ge_fd);

	if (file == NULL) {
		GE_PERR("fail, invalid ge_fd %d\n", ge_fd);
		return -EFAULT;
	}
	if (file->f_op != &GEEntry_fops) {
		GE_PERR("fail, invalid ge_fd %d\n", ge_fd);
		fput(file);
		return -EFAULT;
	}

	entry = file->private_data;

	if (valid_parameters(entry, region_id, u32_offset, u32_size)) {
		err = -EFAULT;
		goto err_parameter;
	}

	spin_lock_irqsave(&ge_raf_lock, flags);
	pregion_data = entry->region_data[region_id];
	spin_unlock_irqrestore(&ge_raf_lock, flags);

	if (pregion_data) {
		for (i = 0; i < u32_size; ++i)
			output_data[i] = pregion_data[u32_offset + i];
	} else {
		for (i = 0; i < u32_size; ++i)
			output_data[i] = 0;
	}

err_parameter:
	fput(file);

	return err;
}

int ged_ge_set(int ge_fd, uint32_t region_id, uint32_t u32_offset,
		uint32_t u32_size, uint32_t *input_data)
{
	unsigned long flags;
	int err = 0;
	uint32_t i;
	GEEntry *entry = NULL;
	uint32_t *pregion_data = NULL;
	struct file *file = fget(ge_fd);

	if (file == NULL) {
		GE_PERR("fail, invalid ge_fd %d\n", ge_fd);
		return -EFAULT;
	}
	if (file->f_op != &GEEntry_fops) {
		GE_PERR("fail, invalid ge_fd %d\n", ge_fd);
		fput(file);
		return -EFAULT;
	}

	entry = file->private_data;

	if (valid_parameters(entry, region_id, u32_offset, u32_size)) {
		err = -EFAULT;
		goto err_parameter;
	}

	spin_lock_irqsave(&ge_raf_lock, flags);
	while (!entry->region_data[region_id]) {
		void *data;

		spin_unlock_irqrestore(&ge_raf_lock, flags);
		data = kzalloc(entry->region_sizes[region_id], GFP_KERNEL);
		if (!data) {
			GE_PERR("kmalloc fail, size: %u\n", entry->region_sizes[region_id]);
			err = -EFAULT;
			goto err_parameter;
		}
		spin_lock_irqsave(&ge_raf_lock, flags);

		/* Check again.
		 * Assign data to entry->region_data[region_id] if it still is NULL.
		 */
		if (entry->region_data[region_id]) {
			spin_unlock_irqrestore(&ge_raf_lock, flags);
			kfree(data);
			spin_lock_irqsave(&ge_raf_lock, flags);
			break;
		}

		entry->region_data[region_id] = data;
	}

	pregion_data = entry->region_data[region_id];
	spin_unlock_irqrestore(&ge_raf_lock, flags);

	for (i = 0; i < u32_size; ++i)
		pregion_data[u32_offset + i] = input_data[i];

err_parameter:
	fput(file);

	return err;
}

int ged_bridge_ge_alloc(GED_BRIDGE_IN_GE_ALLOC *psALLOC_IN,	GED_BRIDGE_OUT_GE_ALLOC *psALLOC_OUT)
{
	psALLOC_OUT->ge_fd = ged_ge_alloc(psALLOC_IN->region_num, psALLOC_IN->region_sizes);
	psALLOC_OUT->eError = psALLOC_OUT->ge_fd >= 0 ? GED_OK : GED_ERROR_OOM;
	return 0;
}

int ged_bridge_ge_get(GED_BRIDGE_IN_GE_GET *psGET_IN, GED_BRIDGE_OUT_GE_GET *psGET_OUT)
{
	psGET_OUT->eError = ged_ge_get(
			psGET_IN->ge_fd,
			psGET_IN->region_id,
			psGET_IN->uint32_offset,
			psGET_IN->uint32_size,
			psGET_OUT->data);
	return 0;
}

int ged_bridge_ge_set(GED_BRIDGE_IN_GE_SET *psSET_IN, GED_BRIDGE_OUT_GE_SET *psSET_OUT)
{
	psSET_OUT->eError = ged_ge_set(
			psSET_IN->ge_fd,
			psSET_IN->region_id,
			psSET_IN->uint32_offset,
			psSET_IN->uint32_size,
			psSET_IN->data);
	return 0;
}

int ged_bridge_ge_info(GED_BRIDGE_IN_GE_INFO *psINFO_IN, GED_BRIDGE_OUT_GE_INFO *psINFO_OUT)
{
	struct file *file = fget(psINFO_IN->ge_fd);

	if (file == NULL) {
		GE_PERR("ged_ge fail, invalid ge_fd %d\n", psINFO_IN->ge_fd);
		return -EFAULT;
	}
	if (file->f_op != &GEEntry_fops) {
		GE_PERR("ged_ge fail, invalid ge_fd %d\n", psINFO_IN->ge_fd);
		fput(file);
		return -EFAULT;
	}

	psINFO_OUT->unique_id = ((GEEntry *)(file->private_data))->unique_id;
	psINFO_OUT->eError = GED_OK;

	fput(file);

	return 0;
}
