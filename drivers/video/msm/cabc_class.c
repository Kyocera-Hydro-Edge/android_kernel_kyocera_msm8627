/* 
 * This software is contributed or developed by KYOCERA Corporation. 
 * (C) 2013 KYOCERA Corporation                                      
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>

#include "cabc_class.h"

static int cabl_mode = 0;
static void cabc_set_enable(struct cabc_dev *dev, int value)
{
  printk("%s(%d) \n", __func__, value);
  cabl_mode = value;
}

static int cabc_get_select(struct cabc_dev *dev)
{
  return(cabl_mode);
}

static struct cabc_dev cabc_set = {
  .name     = "cabc_set",
  .enable   = cabc_set_enable,
  .get_cabc = cabc_get_select,
};

static struct class *cabc_class;
static atomic_t device_count;

static ssize_t enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct cabc_dev *tdev = dev_get_drvdata(dev);
  int remaining = tdev->get_cabc(tdev);

  return sprintf(buf, "%d\n", remaining);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct cabc_dev *tdev = dev_get_drvdata(dev);
  int value;

  if (sscanf(buf, "%d", &value) != 1){
    return -EINVAL;
  }

  tdev->enable(tdev, value);

  return size;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);

static int create_cabc_class(void)
{
  if (!cabc_class) {
    cabc_class = class_create(THIS_MODULE, "cabc");
    if (IS_ERR(cabc_class)){
      return PTR_ERR(cabc_class);
    }
    atomic_set(&device_count, 0);
  }

  cabc_dev_register(&cabc_set);
  return 0;
}

int cabc_dev_register(struct cabc_dev *tdev)
{
  int ret;

  if (!tdev || !tdev->name || !tdev->enable || !tdev->get_cabc){
    return -EINVAL;
  }

  tdev->index = atomic_inc_return(&device_count);
  tdev->dev = device_create(cabc_class, NULL, MKDEV(0, tdev->index), NULL, tdev->name);
  if (IS_ERR(tdev->dev)){
    return PTR_ERR(tdev->dev);
  }

  ret = device_create_file(tdev->dev, &dev_attr_enable);
  if (ret < 0){
    goto err_create_file;
  }

  dev_set_drvdata(tdev->dev, tdev);
  tdev->state = 0;
  return 0;

err_create_file:
  device_destroy(cabc_class, MKDEV(0, tdev->index));
  printk(KERN_ERR "cabc: Failed to register driver %s\n", tdev->name);

  return ret;
}
EXPORT_SYMBOL_GPL(cabc_dev_register);

void cabc_dev_unregister(struct cabc_dev *tdev)
{
  device_remove_file(tdev->dev, &dev_attr_enable);
  device_destroy(cabc_class, MKDEV(0, tdev->index));
  dev_set_drvdata(tdev->dev, NULL);
}
EXPORT_SYMBOL_GPL(cabc_dev_unregister);

static int __init cabc_init(void)
{
  return create_cabc_class();
}

static void __exit cabc_exit(void)
{
  class_destroy(cabc_class);
}

module_init(cabc_init);
module_exit(cabc_exit);

MODULE_AUTHOR("kyocera");
MODULE_DESCRIPTION("cabc class driver");
MODULE_LICENSE("GPL");
