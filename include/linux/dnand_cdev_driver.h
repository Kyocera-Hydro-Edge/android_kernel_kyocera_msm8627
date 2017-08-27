#ifndef DNAND_CDEV_DRIVER_H
#define DNAND_CDEV_DRIVER_H
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */
#include <linux/types.h>
#include "dnand_clid.h"
#include "dnand_status.h"

#define DNAND_CDEV_DRIVER_IOCTL_01    (0x10)
#define DNAND_CDEV_DRIVER_IOCTL_02    (0x11)
#define DNAND_CDEV_DRIVER_IOCTL_03    (0x12)
#define DNAND_CDEV_DRIVER_IOCTL_04    (0x13)

typedef struct dnand_data_type_struct
{
    uint32_t  cid;
    uint32_t  offset;
    uint8_t   *pbuf;
    uint32_t  size;
}dnand_data_type;

int32_t kdnand_id_read( uint32_t id_no, uint32_t offset, uint8_t *pbuf, uint32_t size );
int32_t kdnand_id_write( uint32_t id_no, uint32_t offset, uint8_t *pbuf, uint32_t size );

#endif
