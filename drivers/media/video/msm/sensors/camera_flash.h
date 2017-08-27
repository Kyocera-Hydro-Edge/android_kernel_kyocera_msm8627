/* Copyright (c) 2009-2012, Code Aurora Forum. All rights reserved.
 *
 * This software is contributed or developed by KYOCERA Corporation.
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
#ifndef __CAMERA_FLASH__
#define __CAMERA_FLASH__

struct camera_flash_dev {
  const char *name;

  void (*enable)(struct camera_flash_dev *sdev, int timeout);

  int (*get)(struct camera_flash_dev *sdev);

  struct device *dev;
  int    index;
  int    state;
};

int camera_flash_flashon( void );
int camera_flash_flashon_main( void );
int camera_flash_flashoff( void );

extern int camera_flash_register(struct camera_flash_dev *tdev);
extern void camera_flash_unregister(struct camera_flash_dev *tdev);

#endif
