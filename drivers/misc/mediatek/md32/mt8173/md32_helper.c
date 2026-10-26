/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/fs.h>           /* needed by file_operations */
#include <linux/init.h>         /* needed by module macros */
#include <linux/interrupt.h>
#include <linux/io.h>             /* needed by ioremap */
#include <linux/module.h>       /* needed by all modules */
#include <linux/miscdevice.h>   /* needed by miscdevice* */
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/poll.h>         /* needed by poll */
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <mach/md32_ipi.h>
#include <mach/md32_helper.h>
#include <mt-plat/sync_write.h>
#include <mt_spm_sleep.h>

#include "md32_irq.h"

#define TIMEOUT	100
#define MD32_DEVICE_NAME	"md32"
#define MD32_DATA_IMAGE_PATH	"/etc/firmware/md32_d.bin"
#define MD32_PROGRAM_IMAGE_PATH	"/etc/firmware/md32_p.bin"

#define MD32_SEMAPHORE	(MD32_BASE + 0x90)

/* This structre need to sync with MD32-side */
struct md32_log_info {
	unsigned int md32_log_buf_addr;
	unsigned int md32_log_start_idx_addr;
	unsigned int md32_log_end_idx_addr;
	unsigned int md32_log_lock_addr;
	unsigned int md32_log_buf_len_addr;
	unsigned int enable_md32_mobile_log_addr;
};

struct md32_regs	md32reg;
static struct clk	*md32_clksys;
unsigned char g_md32_log_buf[1024+1];
unsigned int md32_mobile_log_ipi_check = 0;
unsigned int md32_mobile_log_ready = 0;
unsigned int buff_full_count = 0;
unsigned int last_buff_full_count = 0;
unsigned int last_log_buf_max_len = 0;

unsigned long md32_log_buf_addr;
unsigned long md32_log_start_idx_addr;
unsigned long md32_log_end_idx_addr;
unsigned long md32_log_lock_addr;
unsigned long md32_log_buf_len_addr;
unsigned long enable_md32_mobile_log_addr;

static wait_queue_head_t logwait;

static void memcpy_md32(void __iomem *trg, const void *src, int size)
{
	int i;
	u32 __iomem *t = trg;
	const u32 *s = src;

	for (i = 0; i < (size >> 2); i++)
		*t++ = *s++;
}

void memcpy_from_md32(void *trg, const void __iomem *src, int size)
{
	int i;
	u32 *t = trg;
	const u32 __iomem *s = src;

	for (i = 0; i < ((size + 3) >> 2); i++)
		*t++ = *s++;
}

int get_md32_semaphore(int flag)
{
	int read_back;
	int count = 0;
	int ret = -1;

	flag = (flag * 2) + 1;

	read_back = (readl(MD32_SEMAPHORE) >> flag) & 0x1;
	if (read_back == 0) {
		writel((1 << flag), MD32_SEMAPHORE);

		while (count != TIMEOUT) {
			/* repeat test if we get semaphore */
			read_back = (readl(MD32_SEMAPHORE) >> flag) & 0x1;
			if (read_back == 1) {
				ret = 1;
				break;
			}
			writel((1 << flag), MD32_SEMAPHORE);
			count++;
		}

		if (ret < 0)
			pr_err("[MD32] get md32 semaphore %d TIMEOUT...!\n",
			       flag);
	} else {
		pr_err("[MD32] try to double get md32 semaphore %d\n", flag);
	}

	return ret;
}

int release_md32_semaphore(int flag)
{
	int read_back;
	int ret = -1;

	flag = (flag * 2) + 1;

	read_back = (readl(MD32_SEMAPHORE) >> flag) & 0x1;
	if (read_back == 1) {
		/* Write 1 clear */
		writel((1 << flag), MD32_SEMAPHORE);
		read_back = (readl(MD32_SEMAPHORE) >> flag) & 0x1;
		if (read_back == 0)
			ret = 1;
		else
			pr_err("[MD32] release md32 semaphore %d failed!\n",
			       flag);
	} else {
		pr_err("[MD32] try to double release md32 semaphore %d\n",
		       flag);
	}

	return ret;
}

void __iomem *md32_get_dtcm(void)
{
	return MD32_DTCM;
}

ssize_t md32_get_log_buf(unsigned char *md32_log_buf, size_t b_len)
{
	ssize_t i = 0;
	unsigned long log_start_idx;
	unsigned long log_end_idx;
	unsigned long log_buf_max_len;
	unsigned char *__log_buf = (unsigned char *)(MD32_DTCM +
				   md32_log_buf_addr);

	log_start_idx = readl(MD32_DTCM + md32_log_start_idx_addr);
	log_end_idx = readl(MD32_DTCM + md32_log_end_idx_addr);
	log_buf_max_len = readl(MD32_DTCM + md32_log_buf_len_addr);

	if (!md32_log_buf) {
		pr_err("[MD32] input null buffer\n");
		goto out;
	}

	if (b_len > log_buf_max_len) {
		pr_warn("[MD32] b_len %zu > log_buf_max_len %lu\n", b_len,
			log_buf_max_len);
		b_len = log_buf_max_len;
	}

#define LOG_BUF_MASK (log_buf_max_len-1)
#define LOG_BUF(idx) (__log_buf[(idx) & LOG_BUF_MASK])

	/* Read MD32 log */
	/* Lock the log buffer */
	mt_reg_sync_writel(0x1, (MD32_DTCM + md32_log_lock_addr));

	i = 0;
	while ((log_start_idx != log_end_idx) && i < b_len) {
		md32_log_buf[i] = LOG_BUF(log_start_idx);
		log_start_idx++;
		i++;
	}
	mt_reg_sync_writel(log_start_idx, (MD32_DTCM +
					   md32_log_start_idx_addr));
	/* Unlock the log buffer */
	mt_reg_sync_writel(0x0, (MD32_DTCM + md32_log_lock_addr));

out:

	return i;
}

static int md32_log_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static ssize_t md32_log_read(struct file *file, char __user *data, size_t len,
			     loff_t *ppos)
{
	int ret_len;
	unsigned int log_buf_max_len;
	unsigned long copy_size;

	log_buf_max_len = readl(MD32_DTCM + md32_log_buf_len_addr);

	if (log_buf_max_len != last_log_buf_max_len)
		last_log_buf_max_len = log_buf_max_len;

	ret_len = md32_get_log_buf(g_md32_log_buf, len);

	if (ret_len) {
		g_md32_log_buf[ret_len] = '\0';
	} else {
		strcpy(g_md32_log_buf, " ");
		ret_len = strlen(g_md32_log_buf);
	}

	copy_size = copy_to_user((unsigned char *)data,
				 (unsigned char *)g_md32_log_buf, ret_len);

	return ret_len;
}

static unsigned int md32_poll(struct file *file, poll_table *wait)
{
	unsigned int log_start_idx;
	unsigned int log_end_idx;
	unsigned int ret = 0; /* POLLOUT | POLLWRNORM; */

	/* printk("[MD32] Enter md32_poll\n"); */
	if (!(file->f_mode & FMODE_READ))
		return ret;

	poll_wait(file, &logwait, wait);

	if (!md32_mobile_log_ipi_check && md32_mobile_log_ready) {
		log_start_idx = readl((MD32_DTCM + md32_log_start_idx_addr));
		log_end_idx = readl((MD32_DTCM + md32_log_end_idx_addr));
		if (log_start_idx != log_end_idx)
			ret |= (POLLIN | POLLRDNORM);
	} else if (last_buff_full_count != buff_full_count) {
		last_buff_full_count++;
		log_start_idx = readl((MD32_DTCM + md32_log_start_idx_addr));
		log_end_idx = readl((MD32_DTCM + md32_log_end_idx_addr));
		if (log_start_idx != log_end_idx)
			ret |= (POLLIN | POLLRDNORM);
	}

	/* printk("[MD32] Exit2 md32_poll, ret = %d\n", ret); */
	return ret;
}

static const struct file_operations md32_file_ops = {
	.owner = THIS_MODULE,
	.read = md32_log_read,
	.open = md32_log_open,
	.poll = md32_poll,
};

static struct miscdevice md32_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MD32_DEVICE_NAME,
	.fops = &md32_file_ops
};

void logger_ipi_handler(int id, void *data, unsigned int len)
{
	struct md32_log_info *log_info = (struct md32_log_info *)data;

	md32_log_buf_addr = log_info->md32_log_buf_addr;
	md32_log_start_idx_addr = log_info->md32_log_start_idx_addr;
	md32_log_end_idx_addr = log_info->md32_log_end_idx_addr;
	md32_log_lock_addr = log_info->md32_log_lock_addr;
	md32_log_buf_len_addr = log_info->md32_log_buf_len_addr;
	enable_md32_mobile_log_addr = log_info->enable_md32_mobile_log_addr;

	md32_mobile_log_ready = 1;

	pr_debug("[MD32] md32_log_buf_addr = %lx\n", md32_log_buf_addr);
	pr_debug("[MD32] md32_log_start_idx_addr = %lx\n",
		 md32_log_start_idx_addr);
	pr_debug("[MD32] md32_log_end_idx_addr = %lx\n", md32_log_end_idx_addr);
	pr_debug("[MD32] md32_log_lock_addr = %lx\n", md32_log_lock_addr);
	pr_debug("[MD32] md32_log_buf_len_addr = %lx\n", md32_log_buf_len_addr);
	pr_debug("[MD32] enable_md32_mobile_log_addr = %lx\n",
		 enable_md32_mobile_log_addr);
}

void buf_full_ipi_handler(int id, void *data, unsigned int len)
{
	buff_full_count += 8;
	wake_up(&logwait);
}

/* Copy an MD32 image into TCM; the part beyond tcm_size is dropped. */
static int md32_load_image(const char *path, void __iomem *tcm,
			   size_t tcm_size)
{
	struct file *filp;
	void *buf;
	size_t size;
	int ret;

	filp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		pr_err("[MD32] Open MD32 image %s FAIL (%ld)!\n", path,
		       PTR_ERR(filp));
		return PTR_ERR(filp);
	}

	size = min_t(loff_t, i_size_read(file_inode(filp)), tcm_size);
	buf = kmalloc(size, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto out_close;
	}

	ret = kernel_read(filp, 0, buf, size);
	if (ret != size) {
		pr_err("[MD32] Read MD32 image %s FAIL (%d)!\n", path, ret);
		ret = ret < 0 ? ret : -EIO;
		goto out_free;
	}

	memcpy_md32(tcm, buf, size);
	ret = 0;
out_free:
	kfree(buf);
out_close:
	filp_close(filp, NULL);
	return ret;
}

void boot_up_md32(void)
{
	mt_reg_sync_writel(0x1, MD32_BASE);
}

static inline ssize_t md32_log_len_show(struct device *kobj,
					struct device_attribute *attr,
					char *buf)
{
	int log_legnth = 0;
	unsigned int log_buf_max_len = 0;

	if (md32_mobile_log_ready) {
		log_buf_max_len = readl((MD32_DTCM + md32_log_buf_len_addr));
		log_legnth = log_buf_max_len;
	}

	return sprintf(buf, "%08x\n", log_legnth);
}

static ssize_t md32_log_len_store(struct device *kobj,
				  struct device_attribute *attr,
				  const char *buf, size_t n)
{
	/*do nothing */
	return n;
}

DEVICE_ATTR(md32_log_len, 0644, md32_log_len_show, md32_log_len_store);

static inline ssize_t md32_boot_show(struct device *kobj,
				     struct device_attribute *attr, char *buf)
{
	unsigned int sw_rstn;

	sw_rstn = readl((void __iomem *)MD32_BASE);

	if (sw_rstn == 0x0)
		return sprintf(buf, "MD32 not enabled\n");
	else
		return sprintf(buf, "MD32 is running...\n");
}

int reboot_load_md32(void)
{
	unsigned int sw_rstn;
	int ret;

	sw_rstn = readl((void __iomem *)MD32_BASE);

	if (sw_rstn == 0x1)
		pr_warn("MD32 is already running, reboot now...\n");

	/* reset MD32 */
	mt_reg_sync_writel(0x0, MD32_BASE);

	ret = md32_load_image(MD32_DATA_IMAGE_PATH, MD32_DTCM, MD32_DTCM_SIZE);
	if (ret) {
		pr_err("MD32 boot up failed --> load data image failed!\n");
		return ret;
	}

	ret = md32_load_image(MD32_PROGRAM_IMAGE_PATH, MD32_PTCM,
			      MD32_PTCM_SIZE);
	if (ret) {
		pr_err("MD32 boot up failed --> load program image failed!\n");
		return ret;
	}

	boot_up_md32();
	return 0;
}

static ssize_t md32_boot_store(struct device *kobj,
			       struct device_attribute *attr, const char *buf,
			       size_t n)
{
	unsigned int enable = 0;
	unsigned int sw_rstn;
	int ret;

	ret = kstrtouint(buf, 10, &enable);
	if (ret) {
		pr_err("[MD32][%s] string parse error", __func__);
		return ret;
	}

	if (enable != 1) {
		pr_err("[MD32] Can't enable MD32\n");
		return -EINVAL;
	}

	sw_rstn = readl(MD32_BASE);
	if (sw_rstn == 0x0) {
		pr_debug("[MD32] Enable MD32\n");
		if (reboot_load_md32() < 0) {
			pr_err("[MD32] Enable MD32 failed\n");
			return -EINVAL;
		}
	} else {
		pr_warn("[MD32] MD32 is enabled\n");
	}

	return n;
}

DEVICE_ATTR(md32_boot, 0644, md32_boot_show, md32_boot_store);

static inline ssize_t md32_mobile_log_show(struct device *kobj,
					   struct device_attribute *attr,
					   char *buf)
{
	unsigned int enable_md32_mobile_log = 0;

	if (md32_mobile_log_ready)
		enable_md32_mobile_log = readl((void __iomem *)(MD32_DTCM +
					       enable_md32_mobile_log_addr));

	if (enable_md32_mobile_log == 0x0)
		return sprintf(buf, "MD32 mobile log is disabled\n");
	else if (enable_md32_mobile_log == 0x1)
		return sprintf(buf, "MD32 mobile log is enabled\n");
	else
		return sprintf(buf, "MD32 mobile log is in unknown state...\n");
}

static ssize_t md32_mobile_log_store(struct device *kobj,
				     struct device_attribute *attr,
				     const char *buf, size_t n)
{
	unsigned int enable;
	int ret;

	ret = kstrtouint(buf, 10, &enable);
	if (ret) {
		pr_err("enable mobile log failed\n");
		return ret;
	}

	if (md32_mobile_log_ready)
		mt_reg_sync_writel(enable,
				   (MD32_DTCM + enable_md32_mobile_log_addr));

	return n;
}

DEVICE_ATTR(md32_mobile_log, 0644, md32_mobile_log_show, md32_mobile_log_store);

static int create_files(void)
{
	int ret = device_create_file(md32_device.this_device,
				     &dev_attr_md32_log_len);

	if (unlikely(ret != 0))
		return ret;

	ret = device_create_file(md32_device.this_device, &dev_attr_md32_boot);
	if (unlikely(ret != 0))
		return ret;

	ret = device_create_file(md32_device.this_device,
				 &dev_attr_md32_mobile_log);
	if (unlikely(ret != 0))
		return ret;

	ret = device_create_file(md32_device.this_device, &dev_attr_md32_ocd);
	if (unlikely(ret != 0))
		return ret;

	return 0;
}

int md32_dt_init(struct platform_device *pdev)
{
	int ret = 0;
	struct md32_regs *mreg = &md32reg;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "get md32 sram memory resource failed.\n");
		ret = -ENXIO;
		return ret;
	}
	mreg->sram = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mreg->sram)) {
		dev_err(&pdev->dev, "devm_ioremap_resource md32 sram failed.\n");
		ret = PTR_ERR(mreg->sram);
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res == NULL) {
		dev_err(&pdev->dev, "get md32 cfg memory resource failed.\n");
		ret = -ENXIO;
		return ret;
	}
	mreg->cfg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mreg->cfg)) {
		dev_err(&pdev->dev, "devm_ioremap_resource vpu cfg failed.\n");
		ret = PTR_ERR(mreg->cfg);
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "get irq resource failed.\n");
		ret = -ENXIO;
		return ret;
	}
	mreg->irq = platform_get_irq(pdev, 0);

	pr_debug("[MD32] md32 sram base=0x%p, cfgreg=0x%p\n",
		 mreg->sram, mreg->cfg);

	return ret;
}

static int md32_pm_event(struct notifier_block *notifier,
			 unsigned long pm_event, void *unused)
{
	int retval;

	switch (pm_event) {
	case PM_POST_HIBERNATION:
		pr_warn("[MD32] md32_pm_event MD32 reboot\n");
		retval = reboot_load_md32();
		if (retval < 0) {
			retval = -EINVAL;
			pr_err("[MD32] md32_pm_event MD32 reboot Fail\n");
		}
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static struct notifier_block md32_pm_notifier_block = {
	.notifier_call = md32_pm_event,
	.priority = 0,
};

static int md32_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev;

	init_waitqueue_head(&logwait);

	pr_debug("[MD32] initialization\n");

	ret = misc_register(&md32_device);
	if (unlikely(ret != 0)) {
		pr_err("[MD32] misc register failed\n");
		ret = -1;
	}

	ret = md32_dt_init(pdev);
	if (ret) {
		pr_err("[MD32] Device Init Fail\n");
		return ret;
	}

	dev = &pdev->dev;

	/*MD32 clock */
	md32_clksys = devm_clk_get(dev, "sys");
	if (IS_ERR(md32_clksys)) {
		pr_err("[MD32] devm_clk_get clksys fail, %d\n",
		       IS_ERR(md32_clksys));
		return -EINVAL;
	}
	/* Enable MD32 clock */
	ret = clk_prepare_enable(md32_clksys);
	if (ret) {
		pr_err("[VPU] enable md32 clksys failed %d\n", ret);
		return ret;
	}

	/* Enable MD32 sram power */
	spm_poweron_config_set();
	spm_md32_sram_con(0xfffffff0);

	md32_irq_init();
	md32_ipi_init();
	md32_ocd_init();

	/* register logger IPI */
	md32_ipi_registration(IPI_LOGGER, logger_ipi_handler, "logger");

	/* register log buf full IPI */
	md32_ipi_registration(IPI_BUF_FULL, buf_full_ipi_handler, "buf_full");

	ret = create_files();
	if (unlikely(ret != 0))
		pr_err("[MD32] create files failed\n");

	ret = register_pm_notifier(&md32_pm_notifier_block);
	if (ret)
		pr_err("[MD32] failed to register PM notifier %d\n", ret);

	ret = devm_request_irq(dev, md32reg.irq, md32_irq_handler, 0,
			       pdev->name, NULL);
	if (ret)
		dev_err(&pdev->dev, "[MD32] failed to request irq\n");

	return ret;
}

static int md32_remove(struct platform_device *pdev)
{
	misc_deregister(&md32_device);
	return 0;
}

static int mtk_md32_suspend(struct device *dev)
{
	clk_disable_unprepare(md32_clksys);

	return 0;
}

static int mtk_md32_resume(struct device *dev)
{
	int ret = clk_prepare_enable(md32_clksys);
	if (ret) {
		dev_err(dev, "[MD32] failed to enable md32 clock at resume\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id md32_match[] = {
	{ .compatible = "mediatek,mt8173-md32",},
	{},
};
MODULE_DEVICE_TABLE(of, md32_match);

static const struct dev_pm_ops mtk_md32_pm = {
	.suspend = mtk_md32_suspend,
	.resume = mtk_md32_resume,
};

static struct platform_driver md32_driver = {
	.probe	= md32_probe,
	.remove	= md32_remove,
	.driver	= {
		.name	= MD32_DEVICE_NAME,
		.pm = &mtk_md32_pm,
		.owner	= THIS_MODULE,
		.of_match_table = md32_match,
	},
};

module_platform_driver(md32_driver);
