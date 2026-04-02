// SPDX-License-Identifier: GPL-2.0-only
/*
 * GTF DFE Optimization - Linux Kernel Module
 * Extends mqnic driver with DFE calibration, eye scan, and mode control.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/io.h>

#include "mqnic_gtf_dfe.h"

#define DRIVER_NAME "mqnic_gtf_dfe"
#define MAX_DEVICES 8

struct gtf_dfe_dev {
	void __iomem *regs;
	struct pci_dev *pdev;
	struct cdev cdev;
	struct device *dev;
	int minor;
	struct mutex lock;
};

static dev_t gtf_dfe_devt;
static struct class *gtf_dfe_class;
static struct gtf_dfe_dev *devices[MAX_DEVICES];
static DEFINE_MUTEX(devices_lock);

/* Register access helpers */
static inline u32 gtf_dfe_read(struct gtf_dfe_dev *d, u32 offset)
{
	return ioread32(d->regs + offset);
}

static inline void gtf_dfe_write(struct gtf_dfe_dev *d, u32 offset, u32 val)
{
	iowrite32(val, d->regs + offset);
}

/* Wait for calibration to complete (with timeout) */
static int gtf_dfe_wait_cal(struct gtf_dfe_dev *d, unsigned long timeout_ms)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	u32 status;

	do {
		status = gtf_dfe_read(d, GTF_DFE_REG_STATUS);
		if (status & GTF_DFE_STATUS_CAL_DONE)
			return 0;
		if (!(status & GTF_DFE_STATUS_CAL_BUSY))
			return 0;
		usleep_range(100, 200);
	} while (time_before(jiffies, deadline));

	return -ETIMEDOUT;
}

/* Wait for eye scan to complete */
static int gtf_dfe_wait_scan(struct gtf_dfe_dev *d, unsigned long timeout_ms)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	u32 status;

	do {
		status = gtf_dfe_read(d, GTF_DFE_REG_STATUS);
		if (status & GTF_DFE_STATUS_EYE_DONE)
			return 0;
		if (!(status & GTF_DFE_STATUS_EYE_BUSY))
			return 0;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));

	return -ETIMEDOUT;
}

/* Wait for DRP transaction */
static int gtf_dfe_wait_drp(struct gtf_dfe_dev *d)
{
	int tries = 1000;
	u32 status;

	do {
		status = gtf_dfe_read(d, GTF_DFE_REG_DRP_STATUS);
		if (!(status & 0x1))  /* not busy */
			return 0;
		udelay(1);
	} while (--tries > 0);

	return -ETIMEDOUT;
}

static int gtf_dfe_do_calibrate(struct gtf_dfe_dev *d)
{
	u32 ctrl;

	ctrl = gtf_dfe_read(d, GTF_DFE_REG_CTRL);
	ctrl |= GTF_DFE_CTRL_CAL_START;
	gtf_dfe_write(d, GTF_DFE_REG_CTRL, ctrl);

	return gtf_dfe_wait_cal(d, 5000);
}

static int gtf_dfe_do_eye_scan(struct gtf_dfe_dev *d)
{
	u32 ctrl;

	ctrl = gtf_dfe_read(d, GTF_DFE_REG_CTRL);
	ctrl |= GTF_DFE_CTRL_EYE_SCAN;
	gtf_dfe_write(d, GTF_DFE_REG_CTRL, ctrl);

	return gtf_dfe_wait_scan(d, 30000);
}

static int gtf_dfe_do_quick_scan(struct gtf_dfe_dev *d)
{
	u32 ctrl;

	ctrl = gtf_dfe_read(d, GTF_DFE_REG_CTRL);
	ctrl |= GTF_DFE_CTRL_QUICK_SCAN;
	gtf_dfe_write(d, GTF_DFE_REG_CTRL, ctrl);

	return gtf_dfe_wait_scan(d, 10000);
}

static void gtf_dfe_get_taps_hw(struct gtf_dfe_dev *d, struct gtf_dfe_taps *taps)
{
	int i;

	taps->count = gtf_dfe_read(d, GTF_DFE_REG_TAP_COUNT) & 0xF;
	taps->active_mask = gtf_dfe_read(d, GTF_DFE_REG_TAP_ACTIVE) & 0x7FFF;

	for (i = 0; i < GTF_DFE_NUM_TAPS; i++)
		taps->values[i] = gtf_dfe_read(d, GTF_DFE_REG_TAP_VAL_BASE + i * 4) & 0x1F;
}

static void gtf_dfe_set_taps_hw(struct gtf_dfe_dev *d, const struct gtf_dfe_taps *taps)
{
	u32 ctrl;
	int i;

	for (i = 0; i < GTF_DFE_NUM_TAPS; i++)
		gtf_dfe_write(d, GTF_DFE_REG_TAP_OVR_BASE + i * 4, taps->values[i] & 0x1F);

	/* Trigger override */
	ctrl = gtf_dfe_read(d, GTF_DFE_REG_CTRL);
	ctrl |= GTF_DFE_CTRL_OVERRIDE;
	gtf_dfe_write(d, GTF_DFE_REG_CTRL, ctrl);
}

static void gtf_dfe_get_status_hw(struct gtf_dfe_dev *d, struct gtf_dfe_status *st)
{
	u32 status;

	st->ctrl = gtf_dfe_read(d, GTF_DFE_REG_CTRL);
	st->status = gtf_dfe_read(d, GTF_DFE_REG_STATUS);
	st->eye_height = gtf_dfe_read(d, GTF_DFE_REG_EYE_HEIGHT) & 0xFFFF;
	st->eye_width = gtf_dfe_read(d, GTF_DFE_REG_EYE_WIDTH) & 0xFFFF;
	st->eye_ber = gtf_dfe_read(d, GTF_DFE_REG_EYE_BER);
	st->tx_align_hits = gtf_dfe_read(d, GTF_DFE_REG_TX_ALIGN_HIT);
	st->tx_align_misses = gtf_dfe_read(d, GTF_DFE_REG_TX_ALIGN_MISS);
	st->active_taps = gtf_dfe_read(d, GTF_DFE_REG_TAP_COUNT) & 0xF;
	st->active_mask = gtf_dfe_read(d, GTF_DFE_REG_TAP_ACTIVE) & 0x7FFF;

	status = st->status;
	if (status & GTF_DFE_STATUS_LPM_ACTIVE)
		st->mode = GTF_DFE_MODE_LPM;
	else if (status & GTF_DFE_STATUS_DFE_ACTIVE)
		st->mode = GTF_DFE_MODE_DFE;
	else
		st->mode = GTF_DFE_MODE_AUTO;

	st->buf_bypass = !!(status & GTF_DFE_STATUS_BUF_ALIGNED);
	st->fcs_check = !!(st->ctrl & GTF_DFE_CTRL_FCS_CHECK);
}

/* ioctl handler */
static long gtf_dfe_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct gtf_dfe_dev *d = file->private_data;
	void __user *uarg = (void __user *)arg;
	int ret = 0;

	mutex_lock(&d->lock);

	switch (cmd) {
	case GTF_DFE_CALIBRATE:
		ret = gtf_dfe_do_calibrate(d);
		break;

	case GTF_DFE_EYE_SCAN:
		ret = gtf_dfe_do_eye_scan(d);
		break;

	case GTF_DFE_QUICK_SCAN:
		ret = gtf_dfe_do_quick_scan(d);
		break;

	case GTF_DFE_GET_TAPS: {
		struct gtf_dfe_taps taps;
		gtf_dfe_get_taps_hw(d, &taps);
		if (copy_to_user(uarg, &taps, sizeof(taps)))
			ret = -EFAULT;
		break;
	}

	case GTF_DFE_SET_TAPS: {
		struct gtf_dfe_taps taps;
		if (copy_from_user(&taps, uarg, sizeof(taps))) {
			ret = -EFAULT;
			break;
		}
		gtf_dfe_set_taps_hw(d, &taps);
		ret = gtf_dfe_wait_cal(d, 1000);
		break;
	}

	case GTF_DFE_SET_MODE: {
		int mode;
		u32 ctrl;
		if (copy_from_user(&mode, uarg, sizeof(mode))) {
			ret = -EFAULT;
			break;
		}
		ctrl = gtf_dfe_read(d, GTF_DFE_REG_CTRL);
		ctrl &= ~(GTF_DFE_CTRL_LPM_FORCE | GTF_DFE_CTRL_DFE_FORCE | GTF_DFE_CTRL_AUTO_MODE);
		switch (mode) {
		case GTF_DFE_MODE_LPM:
			ctrl |= GTF_DFE_CTRL_LPM_FORCE;
			break;
		case GTF_DFE_MODE_DFE:
			ctrl |= GTF_DFE_CTRL_DFE_FORCE;
			break;
		case GTF_DFE_MODE_AUTO:
			ctrl |= GTF_DFE_CTRL_AUTO_MODE;
			break;
		default:
			ret = -EINVAL;
		}
		if (!ret)
			gtf_dfe_write(d, GTF_DFE_REG_CTRL, ctrl);
		break;
	}

	case GTF_DFE_GET_STATUS: {
		struct gtf_dfe_status st;
		gtf_dfe_get_status_hw(d, &st);
		if (copy_to_user(uarg, &st, sizeof(st)))
			ret = -EFAULT;
		break;
	}

	case GTF_DFE_DRP_READ: {
		struct gtf_dfe_drp_xfer xfer;
		if (copy_from_user(&xfer, uarg, sizeof(xfer))) {
			ret = -EFAULT;
			break;
		}
		gtf_dfe_write(d, GTF_DFE_REG_DRP_ADDR, xfer.addr);
		gtf_dfe_write(d, GTF_DFE_REG_DRP_CTRL, 0x01);  /* start read */
		ret = gtf_dfe_wait_drp(d);
		if (!ret) {
			xfer.data = gtf_dfe_read(d, GTF_DFE_REG_DRP_DATA) & 0xFFFF;
			if (copy_to_user(uarg, &xfer, sizeof(xfer)))
				ret = -EFAULT;
		}
		break;
	}

	case GTF_DFE_DRP_WRITE: {
		struct gtf_dfe_drp_xfer xfer;
		if (copy_from_user(&xfer, uarg, sizeof(xfer))) {
			ret = -EFAULT;
			break;
		}
		gtf_dfe_write(d, GTF_DFE_REG_DRP_ADDR, xfer.addr);
		gtf_dfe_write(d, GTF_DFE_REG_DRP_DATA, xfer.data);
		gtf_dfe_write(d, GTF_DFE_REG_DRP_CTRL, 0x03);  /* start write */
		ret = gtf_dfe_wait_drp(d);
		break;
	}

	default:
		ret = -ENOTTY;
	}

	mutex_unlock(&d->lock);
	return ret;
}

static int gtf_dfe_open(struct inode *inode, struct file *file)
{
	struct gtf_dfe_dev *d = container_of(inode->i_cdev, struct gtf_dfe_dev, cdev);
	file->private_data = d;
	return 0;
}

static int gtf_dfe_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations gtf_dfe_fops = {
	.owner = THIS_MODULE,
	.open = gtf_dfe_open,
	.release = gtf_dfe_release,
	.unlocked_ioctl = gtf_dfe_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

/* sysfs attributes */
static ssize_t mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gtf_dfe_dev *d = dev_get_drvdata(dev);
	u32 status = gtf_dfe_read(d, GTF_DFE_REG_STATUS);

	if (status & GTF_DFE_STATUS_LPM_ACTIVE)
		return sysfs_emit(buf, "lpm\n");
	else if (status & GTF_DFE_STATUS_DFE_ACTIVE)
		return sysfs_emit(buf, "dfe\n");
	else
		return sysfs_emit(buf, "unknown\n");
}
static DEVICE_ATTR_RO(mode);

static ssize_t active_taps_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gtf_dfe_dev *d = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", gtf_dfe_read(d, GTF_DFE_REG_TAP_COUNT) & 0xF);
}
static DEVICE_ATTR_RO(active_taps);

static ssize_t eye_height_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gtf_dfe_dev *d = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", gtf_dfe_read(d, GTF_DFE_REG_EYE_HEIGHT) & 0xFFFF);
}
static DEVICE_ATTR_RO(eye_height);

static ssize_t eye_width_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gtf_dfe_dev *d = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", gtf_dfe_read(d, GTF_DFE_REG_EYE_WIDTH) & 0xFFFF);
}
static DEVICE_ATTR_RO(eye_width);

static ssize_t buf_bypass_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gtf_dfe_dev *d = dev_get_drvdata(dev);
	u32 status = gtf_dfe_read(d, GTF_DFE_REG_STATUS);
	return sysfs_emit(buf, "%s\n",
		(status & GTF_DFE_STATUS_BUF_ALIGNED) ? "aligned" : "disabled");
}
static DEVICE_ATTR_RO(buf_bypass);

static ssize_t tx_align_hits_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gtf_dfe_dev *d = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", gtf_dfe_read(d, GTF_DFE_REG_TX_ALIGN_HIT));
}
static DEVICE_ATTR_RO(tx_align_hits);

static ssize_t tx_align_misses_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gtf_dfe_dev *d = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%u\n", gtf_dfe_read(d, GTF_DFE_REG_TX_ALIGN_MISS));
}
static DEVICE_ATTR_RO(tx_align_misses);

static struct attribute *gtf_dfe_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_active_taps.attr,
	&dev_attr_eye_height.attr,
	&dev_attr_eye_width.attr,
	&dev_attr_buf_bypass.attr,
	&dev_attr_tx_align_hits.attr,
	&dev_attr_tx_align_misses.attr,
	NULL,
};

static const struct attribute_group gtf_dfe_attr_group = {
	.name = "gtf_dfe",
	.attrs = gtf_dfe_attrs,
};

/* Probe: called when mqnic detects our app block magic */
int mqnic_gtf_dfe_probe(struct pci_dev *pdev, void __iomem *app_regs)
{
	struct gtf_dfe_dev *d;
	u32 version;
	int minor, ret;

	version = ioread32(app_regs + GTF_DFE_REG_VERSION);
	if (version != GTF_DFE_VERSION_MAGIC) {
		dev_dbg(&pdev->dev, "GTF DFE: version mismatch (got 0x%08x)\n", version);
		return -ENODEV;
	}

	/* Scratch register test */
	iowrite32(0xA5A5A5A5, app_regs + GTF_DFE_REG_SCRATCH);
	if (ioread32(app_regs + GTF_DFE_REG_SCRATCH) != 0xA5A5A5A5) {
		dev_err(&pdev->dev, "GTF DFE: scratch register test failed\n");
		return -EIO;
	}
	iowrite32(0, app_regs + GTF_DFE_REG_SCRATCH);

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->regs = app_regs;
	d->pdev = pdev;
	mutex_init(&d->lock);

	/* Find free minor */
	mutex_lock(&devices_lock);
	for (minor = 0; minor < MAX_DEVICES; minor++) {
		if (!devices[minor])
			break;
	}
	if (minor >= MAX_DEVICES) {
		mutex_unlock(&devices_lock);
		kfree(d);
		return -ENOSPC;
	}
	d->minor = minor;
	devices[minor] = d;
	mutex_unlock(&devices_lock);

	/* Create char device */
	cdev_init(&d->cdev, &gtf_dfe_fops);
	d->cdev.owner = THIS_MODULE;
	ret = cdev_add(&d->cdev, MKDEV(MAJOR(gtf_dfe_devt), minor), 1);
	if (ret)
		goto err_cdev;

	d->dev = device_create(gtf_dfe_class, &pdev->dev,
			       MKDEV(MAJOR(gtf_dfe_devt), minor),
			       d, "gtf_dfe%d", minor);
	if (IS_ERR(d->dev)) {
		ret = PTR_ERR(d->dev);
		goto err_device;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &gtf_dfe_attr_group);
	if (ret)
		goto err_sysfs;

	dev_info(&pdev->dev, "GTF DFE optimization module registered (minor %d)\n", minor);
	return 0;

err_sysfs:
	device_destroy(gtf_dfe_class, MKDEV(MAJOR(gtf_dfe_devt), minor));
err_device:
	cdev_del(&d->cdev);
err_cdev:
	mutex_lock(&devices_lock);
	devices[minor] = NULL;
	mutex_unlock(&devices_lock);
	kfree(d);
	return ret;
}
EXPORT_SYMBOL(mqnic_gtf_dfe_probe);

void mqnic_gtf_dfe_remove(struct pci_dev *pdev)
{
	int i;

	mutex_lock(&devices_lock);
	for (i = 0; i < MAX_DEVICES; i++) {
		if (devices[i] && devices[i]->pdev == pdev) {
			sysfs_remove_group(&pdev->dev.kobj, &gtf_dfe_attr_group);
			device_destroy(gtf_dfe_class, MKDEV(MAJOR(gtf_dfe_devt), i));
			cdev_del(&devices[i]->cdev);
			kfree(devices[i]);
			devices[i] = NULL;
		}
	}
	mutex_unlock(&devices_lock);
}
EXPORT_SYMBOL(mqnic_gtf_dfe_remove);

static int __init gtf_dfe_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&gtf_dfe_devt, 0, MAX_DEVICES, DRIVER_NAME);
	if (ret)
		return ret;

	gtf_dfe_class = class_create(DRIVER_NAME);
	if (IS_ERR(gtf_dfe_class)) {
		unregister_chrdev_region(gtf_dfe_devt, MAX_DEVICES);
		return PTR_ERR(gtf_dfe_class);
	}

	pr_info("GTF DFE optimization driver loaded\n");
	return 0;
}

static void __exit gtf_dfe_exit(void)
{
	class_destroy(gtf_dfe_class);
	unregister_chrdev_region(gtf_dfe_devt, MAX_DEVICES);
	pr_info("GTF DFE optimization driver unloaded\n");
}

module_init(gtf_dfe_init);
module_exit(gtf_dfe_exit);

MODULE_AUTHOR("GTF DFE Optimization Project");
MODULE_DESCRIPTION("GTF transceiver DFE optimization for Corundum NIC");
MODULE_LICENSE("GPL");
