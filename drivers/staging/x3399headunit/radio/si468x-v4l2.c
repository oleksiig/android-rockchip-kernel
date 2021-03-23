/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *  Driver for Si468x AM/FM/DAB Radio Receiver on x3399headunit board
 *
 *  Copyright (C) 2021 Oleksii Gulchenko <alexey.gulchenko@gmail.com>
 */

#include "si468x.h"

#define RADIOC_START_BAND   _IOWR('R', 110, struct si468x_band_desc*)
#define RADIOC_STOP         _IOWR('R', 111, int)
#define RADIOC_TUNE         _IOWR('R', 112, uint32_t)
#define RADIOC_SCAN         _IOWR('R', 113, int)
#define     RADIO_SCAN_UP   1
#define     RADIO_SCAN_DOWN 0

static ssize_t si468x_fops_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
    struct si468x_device *radio = video_drvdata(file);
    struct device *dev = &radio->client->dev;
    int ret;
    size_t to_copy, avail_copy = sizeof(struct si468x_tune_status);

    if (file->f_flags & O_NONBLOCK) {
        return -EWOULDBLOCK;
    }

    if(radio->mode == SI468X_MODE_UNKNOWN ||
       radio->mode == SI468X_MODE_BOOT)
    {
        /* No data available */
        return 0;
    }

    mutex_lock(&radio->irq_flags_lock);
    radio->status.flags = radio->irq_flags;
    radio->irq_flags &= ~TUNE_STATUS_FLAG_STC;
    mutex_unlock(&radio->irq_flags_lock);

    memset(&radio->status.rsq, 0, sizeof(struct si468x_rsq_status));
    memset(&radio->status.acf, 0, sizeof(struct si468x_acf_status));
    memset(&radio->status.rds, 0, sizeof(struct si468x_rds_status));

    ret = si468x_read_rsq_status(radio);
    if(ret) {
        dev_err(dev, "Read RSQ status failed, ret %d\n", ret);
        return ret;
    }

    if (radio->mode == SI468X_MODE_FM)
    {
        ret = si468x_fm_read_acf_status(radio);
        if(ret) {
            dev_err(dev, "Read ACF status failed, ret %d\n", ret);
            return ret;
        }

        if(radio->status.flags & TUNE_STATUS_FLAG_RDS) {
            ret = si468x_fm_read_rds_status(radio);
            if(ret && ret != -ETIME) {
                dev_err(dev, "Read RDS status failed, ret %d\n", ret);
                return ret;
            }
            if(ret) { /* if timeout, no RDS */
                radio->status.flags &= ~TUNE_STATUS_FLAG_RDS;
            }
        }
    }

    to_copy = count < avail_copy ? count : avail_copy;

    if(count != to_copy) {
        dev_warn(dev, "Copied data %zu, but avail %zu \n",
            count, avail_copy);
    }

    if (copy_to_user(buf, &radio->status, to_copy)) {
		dev_err(dev, "copy_to_user failed\n");
        return -EFAULT;
    }

    return to_copy;
}

static __poll_t si468x_fops_poll(struct file *file,
		struct poll_table_struct *pts)
{
    struct si468x_device *radio = video_drvdata(file);
	__poll_t req_events = poll_requested_events(pts);
	__poll_t retval = 0;
    int ret;

    if (req_events & EPOLLIN)
    {
        ret = wait_event_interruptible_timeout(radio->status_queue,
            (radio->irq_flags & TUNE_STATUS_FLAG_STC), 1*HZ);

        retval |= EPOLLIN;
	}

	return retval;
}

static int si468x_fops_open(struct file *file)
{
	return v4l2_fh_open(file);
}

static int si468x_fops_release(struct file *file)
{
	struct si468x_device *radio = video_drvdata(file);

    if(radio->fops_release) {
        radio->fops_release(file);
    }

    return v4l2_fh_release(file);
}

long int si468x_fops_ioctl(struct file *file,
		      unsigned int cmd, unsigned long int arg)
{
    struct si468x_device *radio = video_drvdata(file);
    struct device *dev = &radio->client->dev;

    switch(cmd) {
        case RADIOC_START_BAND:
        {
            struct si468x_band_desc band_desc = {0};
            if (copy_from_user(&band_desc, (void*)arg, sizeof(band_desc))) {
                dev_err(dev, "copy_from_user failed\n");
                return -EFAULT;
            }

            switch(band_desc.modulation)
            {
                case RADIO_MODULATION_FM:
                    return si468x_start_fm(radio, &band_desc);
                case RADIO_MODULATION_AM:
                    return si468x_start_am(radio, &band_desc);
                case RADIO_MODULATION_DAB:
                    return si468x_start_dab(radio, &band_desc);
            }
            dev_err(dev, "%s: invalid band %d\n",
                        __func__, band_desc.modulation);
            return -EINVAL;
        }
        case RADIOC_TUNE:
            return si468x_tune(radio, (int)arg);

        case RADIOC_SCAN:
            return si468x_seek_start(radio, ((int)arg == RADIO_SCAN_UP), 1/*wrap*/);

        default:
            dev_err(dev, "%s: unknown cmd 0x%08x\n", __func__, cmd);
    }

    return 0;
}

static const struct v4l2_file_operations si468x_fops = {
	.owner			            = THIS_MODULE,
	.read			            = si468x_fops_read,
	.poll			            = si468x_fops_poll,
	.unlocked_ioctl		        = si468x_fops_ioctl,
	.open			            = si468x_fops_open,
	.release		            = si468x_fops_release,
};

const struct video_device si468x_viddev = {
	.name			            = DRIVER_NAME,
	.release		            = video_device_release_empty,
    .fops			            = &si468x_fops,
};
