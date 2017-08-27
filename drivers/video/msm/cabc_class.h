/*
 * This software is contributed or developed by KYOCERA Corporation. 
 * (C) 2013 KYOCERA Corporation                                      
 */

#ifndef _LINUX_CABC_CLASS_H
#define _LINUX_CABC_CLASS_H

struct cabc_dev {
  const char *name;

  void (*enable)(struct cabc_dev *sdev, int timeout);

  int (*get_cabc)(struct cabc_dev *sdev);

  struct device *dev;
  int    index;
  int    state;
};

extern int cabc_dev_register(struct cabc_dev *dev);
extern void cabc_dev_unregister(struct cabc_dev *dev);

#endif
