/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <mach/pmic.h>
#include <mach/camera.h>
#include <mach/gpio.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>

#include "msm_sensor.h"
#include "camera_flash.h"

#define DBG_LOG_SWITCH
#undef DBG_LOG_SWITCH
#ifdef DBG_LOG_SWITCH
#define DBG_LOG(fmt, args...) printk(KERN_NOTICE fmt, ##args)
#else
#define DBG_LOG(fmt, args...) do{ } while(0)
#endif

#define DBG_CODE_SWITCH
#undef DBG_CODE_SWITCH

#define CAMERA_FLASH_GPIO_EN_DIM_NUM 74
#define CAMERA_FLASH_GPIO_MODE_NUM   75

#define CAMERA_FLASH_GPIO_EN_DIM GPIO_CFG(CAMERA_FLASH_GPIO_EN_DIM_NUM, \
                                          0, \
                                          GPIO_CFG_OUTPUT, \
                                          GPIO_CFG_NO_PULL, \
                                          GPIO_CFG_2MA)

#define CAMERA_FLASH_GPIO_MODE   GPIO_CFG(CAMERA_FLASH_GPIO_MODE_NUM, \
                                          0, \
                                          GPIO_CFG_OUTPUT, \
                                          GPIO_CFG_NO_PULL, \
                                          GPIO_CFG_2MA)

static struct class *msm_camera_flash_class;

static spinlock_t flash_pulse_lock;

static bool work_enabled = false;
static void camera_flash_flashoff_worker(struct work_struct *work);
static DECLARE_DELAYED_WORK(camera_flash_flashoff_work, camera_flash_flashoff_worker);

static bool is_flash_on = false;

int camera_flash_flashon(void)
{
  int cnt = 0;
  unsigned long flags;

  DBG_LOG("%s() E \n", __func__);

  gpio_direction_output(CAMERA_FLASH_GPIO_MODE_NUM,0);

  if(is_flash_on == true)
  {
    camera_flash_flashoff();
  }

  cnt = 2;

  spin_lock_irqsave(&flash_pulse_lock, flags);

  while(cnt > 0)
  {
    cnt--;
    DBG_LOG("%s() : Pulse cnt = [%d]\n", __func__, cnt);
    gpio_direction_output(CAMERA_FLASH_GPIO_EN_DIM_NUM,1);
    gpio_direction_output(CAMERA_FLASH_GPIO_EN_DIM_NUM,0);
  }
  gpio_direction_output(CAMERA_FLASH_GPIO_EN_DIM_NUM,1);

  is_flash_on = true;
  spin_unlock_irqrestore(&flash_pulse_lock, flags);

  DBG_LOG("%s:Final Pulse(High):%d \n", __func__, cnt+1);

  DBG_LOG("%s() X \n", __func__);

  return 0;
}

int camera_flash_flashon_main(void)
{
  int cnt = 0;
  unsigned long flags;

  DBG_LOG("%s() E \n", __func__);

  gpio_direction_output(CAMERA_FLASH_GPIO_MODE_NUM,1);

  if(is_flash_on == true)
  {
    camera_flash_flashoff();
  }

  spin_lock_irqsave(&flash_pulse_lock, flags);

  cnt = 3;

  while(cnt > 0)
  {
    cnt--;
    DBG_LOG("%s() : Pulse cnt = [%d]\n", __func__, cnt);
    gpio_direction_output(CAMERA_FLASH_GPIO_EN_DIM_NUM,1);
    gpio_direction_output(CAMERA_FLASH_GPIO_EN_DIM_NUM,0);
  }
  gpio_direction_output(CAMERA_FLASH_GPIO_EN_DIM_NUM,1);

  is_flash_on = true;
  spin_unlock_irqrestore(&flash_pulse_lock, flags);

  DBG_LOG("%s:Final Pulse(High):%d \n", __func__, cnt+1);

  work_enabled = true;
  schedule_delayed_work(&camera_flash_flashoff_work,msecs_to_jiffies(1000));

  DBG_LOG("%s() X \n", __func__);

  return 0;
}

int camera_flash_flashoff(void)
{
  DBG_LOG("%s() E \n", __func__);

  if(work_enabled == true)
  {
    DBG_LOG("%s:cancel_delayed_work \n", __func__);
    work_enabled = false;
    cancel_delayed_work(&camera_flash_flashoff_work);
  }
  gpio_direction_output(CAMERA_FLASH_GPIO_EN_DIM_NUM,0);
  gpio_direction_output(CAMERA_FLASH_GPIO_MODE_NUM,0);
  usleep_range(500,500);

  is_flash_on = false;

  DBG_LOG("%s() X \n", __func__);
  return 0;
}

static void camera_flash_flashoff_worker(struct work_struct *work)
{
  DBG_LOG("%s() E \n", __func__);

  work_enabled = false;
  camera_flash_flashoff();

  DBG_LOG("%s() X \n", __func__);
}

static int cabl_mode = 0;
static void camera_flash_set_enable(struct camera_flash_dev *dev, int value)
{
    DBG_LOG("%s(%d) E \n", __func__, value);

    if( value == 1 )
    {
        camera_flash_flashon();
    }
    else
    {
        camera_flash_flashoff();
    }

    DBG_LOG("%s(%d) X \n", __func__, value);
}

static int camera_flash_get_select(struct camera_flash_dev *dev)
{
  DBG_LOG("%s() \n", __func__);
  return(cabl_mode);
}

static struct camera_flash_dev camera_flash_set = {
  .name     = "camera_flash_set",
  .enable   = camera_flash_set_enable,
  .get      = camera_flash_get_select,
};

static atomic_t device_count;

static ssize_t camera_flash_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct camera_flash_dev *tdev = dev_get_drvdata(dev);
  int remaining = tdev->get(tdev);

  DBG_LOG("%s E \n", __func__);

  return sprintf(buf, "%d\n", remaining);
}

#ifdef DBG_CODE_SWITCH
static int camera_flash_mode_ex(int mode);
static int camera_flash_flashon_ex(int cur, int delay);
static int camera_flash_flashoff_ex(void);
#endif

static ssize_t camera_flash_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
#ifdef DBG_CODE_SWITCH
  int val1,val2,val3;

  DBG_LOG("%s() E \n", __func__);

  sscanf(buf, "%d %d %d", &val1, &val2, &val3);

  if((val1 == 0) || (val1 == 1))
  {
    camera_flash_mode_ex(val1);
    camera_flash_flashoff_ex();
    if(val2 > 0)
    {
      camera_flash_flashon_ex(val2, val3);
    }
  }

  DBG_LOG("%s() X \n", __func__);
#else
  struct camera_flash_dev *tdev = dev_get_drvdata(dev);
  int value;
  
  if (sscanf(buf, "%d", &value) != 1){
    return -EINVAL;
  }

  tdev->enable(tdev, value);
#endif
  return size;
}

#ifdef DBG_CODE_SWITCH
static int camera_flash_mode_ex(int mode)
{
  DBG_LOG("%s() E \n", __func__);

  gpio_direction_output(CAMERA_FLASH_GPIO_MODE_NUM,(unsigned char)mode);

  DBG_LOG("%s() X \n", __func__);
  return 0;
}

static int camera_flash_flashon_ex(int cur, int delay)
{
  int cnt = 0;
  unsigned long flags;

  DBG_LOG("%s() E \n", __func__);

  spin_lock_irqsave(&flash_pulse_lock, flags);

  while(cnt < (cur-1))
  {
    cnt++;
    gpio_direction_output(CAMERA_FLASH_GPIO_EN_DIM_NUM,1);
    gpio_direction_output(CAMERA_FLASH_GPIO_EN_DIM_NUM,0);
  }
  gpio_direction_output(CAMERA_FLASH_GPIO_EN_DIM_NUM,1);

  spin_unlock_irqrestore(&flash_pulse_lock, flags);

  DBG_LOG("%s:Final Pulse(High):%d \n", __func__, cnt+1);
  work_enabled = true;
  schedule_delayed_work(&camera_flash_flashoff_work,msecs_to_jiffies(delay));

  DBG_LOG("%s() X \n", __func__);

  return 0;
}

static int camera_flash_flashoff_ex(void)
{
  DBG_LOG("%s() E \n", __func__);

  if(work_enabled == true)
  {
    DBG_LOG("%s:cancel_delayed_work \n", __func__);
    work_enabled = false;
    cancel_delayed_work(&camera_flash_flashoff_work);
  }
  gpio_direction_output(CAMERA_FLASH_GPIO_EN_DIM_NUM,0);
  usleep_range(500,500);

  DBG_LOG("%s() X \n", __func__);
  return 0;
}
#endif

static DEVICE_ATTR(camera_flash, S_IRUGO | S_IWUSR, camera_flash_show, camera_flash_store);

static int create_camera_flash_class(void)
{
  if (!msm_camera_flash_class) {
    msm_camera_flash_class = class_create(THIS_MODULE, "msm_camera_flash");
    if (IS_ERR(msm_camera_flash_class)){
      return PTR_ERR(msm_camera_flash_class);
    }
    atomic_set(&device_count, 0);

    gpio_request(CAMERA_FLASH_GPIO_MODE_NUM,"CAM_FL_MD");
    gpio_request(CAMERA_FLASH_GPIO_EN_DIM_NUM,"CAM_FL_ENDIM");

    gpio_tlmm_config(CAMERA_FLASH_GPIO_MODE,GPIO_CFG_ENABLE);
    gpio_tlmm_config(CAMERA_FLASH_GPIO_EN_DIM,GPIO_CFG_ENABLE);
  }

  camera_flash_register(&camera_flash_set);

  spin_lock_init(&flash_pulse_lock);

  return 0;
}

int camera_flash_register(struct camera_flash_dev *tdev)
{
  int ret;

  if (!tdev || !tdev->name || !tdev->enable || !tdev->get){
    return -EINVAL;
  }

  tdev->index = atomic_inc_return(&device_count);
  tdev->dev = device_create(msm_camera_flash_class, NULL, MKDEV(0, tdev->index), NULL, tdev->name);
  if (IS_ERR(tdev->dev)){
    return PTR_ERR(tdev->dev);
  }

  ret = device_create_file(tdev->dev, &dev_attr_camera_flash);
  if (ret < 0){
    goto err_create_file;
  }

  dev_set_drvdata(tdev->dev, tdev);
  tdev->state = 0;
  return 0;

err_create_file:
  device_destroy(msm_camera_flash_class, MKDEV(0, tdev->index));
  pr_err(KERN_ERR "cabc: Failed to register driver %s\n", tdev->name);

  return ret;
}
EXPORT_SYMBOL_GPL(camera_flash_register);

void camera_flash_unregister(struct camera_flash_dev *tdev)
{
  device_remove_file(tdev->dev, &dev_attr_camera_flash);
  device_destroy(msm_camera_flash_class, MKDEV(0, tdev->index));
  dev_set_drvdata(tdev->dev, NULL);
}
EXPORT_SYMBOL_GPL(camera_flash_unregister);

static int __init camera_flash_init(void)
{
  return create_camera_flash_class();
}

static void __exit camera_flash_exit(void)
{
  class_destroy(msm_camera_flash_class);
}

module_init(camera_flash_init);
module_exit(camera_flash_exit);

MODULE_AUTHOR("kyocera");
MODULE_DESCRIPTION("camera_flash class driver");
MODULE_LICENSE("GPL");

