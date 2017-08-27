#ifndef DNAND_STATUS_H
#define DNAND_STATUS_H
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */
#include <linux/types.h>

#define DNAND_NO_ERROR        (0)
#define DNAND_DEV_ERROR       (-1)
#define DNAND_MNG_ERROR       (-10)
#define DNAND_NOMEM_ERROR     (-20)
#define DNAND_PARAM_ERROR     (-30)
#define DNAND_NOSPC_ERROR     (-40)
#define DNAND_INTERNAL_ERROR  (-50)
#define DNAND_NOEXISTS_ERROR  (-60)
#define DNAND_EOF_ERROR       (-70)

#endif
