/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/dnand_cdev_driver.h>

#include <linux/dnand_clid.h>
#include "dnand_fs.h"
#include "dnand_fs_mng.h"


static DEFINE_MUTEX(dnand_lock);

static int32_t inter_param_check(
            uint32_t            cid,
            uint32_t            offset,
            uint8_t             *pbuf,
            uint32_t            size )
{
    int32_t  rtn;

    rtn    = DNAND_NO_ERROR;
    if( cid >= DNAND_ID_ENUM_MAX )
    {
        rtn    = DNAND_PARAM_ERROR;
    }
    if( pbuf == NULL )
    {
        rtn    = DNAND_PARAM_ERROR;
    }
    if( size == 0 )
    {
        rtn    = DNAND_PARAM_ERROR;
    }
    return(rtn);
}

int32_t        kdnand_id_read(
                uint32_t            cid,
                uint32_t            offset,
                uint8_t             *pbuf,
                uint32_t            size )
{
    int32_t      rtn;

    rtn    = inter_param_check( cid, offset, pbuf, size );
    if( rtn != DNAND_NO_ERROR )
    {
        if( size == 0 )
        {
            rtn    = DNAND_NO_ERROR;
        }
        return(rtn);
    }

    mutex_lock(&dnand_lock);

    rtn    = dnand_fs_read(cid, offset, pbuf, size);

    mutex_unlock(&dnand_lock);

    return(rtn);
}

int32_t        kdnand_id_write(
                uint32_t            cid,
                uint32_t            offset,
                uint8_t             *pbuf,
                uint32_t            size )
{
    int32_t      rtn;

    rtn    = inter_param_check( cid, offset, pbuf, size );
    if( rtn != DNAND_NO_ERROR )
    {
        if( size == 0 )
        {
            rtn    = DNAND_NO_ERROR;
        }
        return(rtn);
    }

    mutex_lock(&dnand_lock);

    rtn    = dnand_fs_write(cid, offset, pbuf, size);

    mutex_unlock(&dnand_lock);

    return(rtn);
}
