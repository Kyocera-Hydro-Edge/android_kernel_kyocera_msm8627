#ifndef DNAND_FS_H
#define DNAND_FS_H
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */
#include <linux/types.h>
#include <linux/dnand_clid.h>
#include <linux/dnand_status.h>

int32_t dnand_fs_read( uint32_t cid, uint32_t offset, uint8_t *pbuf, uint32_t size );
int32_t dnand_fs_write( uint32_t cid, uint32_t offset, uint8_t *pbuf, uint32_t size );

#endif // DNAND_FS_H
