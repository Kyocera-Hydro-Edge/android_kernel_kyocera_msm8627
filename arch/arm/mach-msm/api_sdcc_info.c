/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
*/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/mmc/host.h>
#include <mach/hs_io_ctl_a.h>
#include <mach/msm_smd.h>

#include <mach/smem_log.h>
#include <mach/gpio.h>
#include <linux/gpio.h>

#include "smd_private.h"
#include "api_sdcc_info.h"
#include "board-8930.h"
#define SD_STATUS_DEBUG(...) printk(__VA_ARGS__)

#include <mach/kc_board.h>

/*****************************/
/* static valiables */
/*****************************/
struct sd_device_t {
	struct miscdevice misc;
};

/*****************************/
/* function */
/*****************************/
static long sd_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	struct sd_status_ioctl_info info;
    unsigned long result = -EFAULT;

	switch(cmd)
	{
		/*---------------------------*/
		/* SD_card_status read       */
		/*---------------------------*/
		case IOCTL_SD_STATUS_READ_CMD:
			result = copy_from_user(&info,(struct sd_status_ioctl_info __user *)arg,sizeof(info));
			SD_STATUS_DEBUG("@@@DEBUG IOCTL_SD_STATUS_READ_CMD\n");

			if ( OEM_get_board() <= OEM_BOARD_WS1_TYPE ) {
				if( 1 == gpio_get_value(106) ) {
					SD_STATUS_DEBUG("@@@DEBUG SDcard IN !!!\n");
					info.card_sts = SD_CARD_IN;
				}
				else {
					SD_STATUS_DEBUG("@@@DEBUG SDcard OUT !!!\n");
					info.card_sts = SD_CARD_OUT;
				}
			}
			else {
				if( 0 == gpio_get_value(106) ) {
					SD_STATUS_DEBUG("@@@DEBUG SDcard IN !!!\n");
					info.card_sts = SD_CARD_IN;
				}
				else {
					SD_STATUS_DEBUG("@@@DEBUG SDcard OUT !!!\n");
					info.card_sts = SD_CARD_OUT;
				}
			}
			result = copy_to_user((struct sd_status_ioctl_info __user *)arg,&info,sizeof(info));
			break;

		default:
			break;
	}
	return 0;
}

static int sd_open(struct inode *ip, struct file *fp)
{
	return nonseekable_open(ip, fp);
}

static int sd_release(struct inode *ip, struct file *fp)
{
	return 0;
}

static const struct file_operations sd_fops = {
	.owner = THIS_MODULE,
	.open = sd_open,
	.unlocked_ioctl = sd_ioctl,
	.release = sd_release,
};

static struct sd_device_t sd_device = {
	.misc = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "sd_status",
		.fops = &sd_fops,
	}
};

static void __exit sd_state_exit(void)
{
	misc_deregister(&sd_device.misc);
}

static int __init sd_state_init(void)
{
	int ret;
	ret = misc_register(&sd_device.misc);
	return ret;
}

module_init(sd_state_init);
module_exit(sd_state_exit);

MODULE_DESCRIPTION("SDcard_status Driver");
MODULE_LICENSE("GPL v2");
