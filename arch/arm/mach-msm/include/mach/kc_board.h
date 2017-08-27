/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */
/* Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
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
#ifndef _KC_BOARD_H_
#define _KC_BOARD_H_

typedef enum {
    OEM_BOARD_WS0_TYPE     = 0,
    OEM_BOARD_WS1_TYPE     = 1,
    OEM_BOARD_WS2_TYPE     = 2,
    OEM_BOARD_LAB_TYPE     = 3,
    OEM_BOARD_UT_TYPE      = 4,
    OEM_BOARD_PP_TYPE      = 5,
    OEM_BOARD_RESERVE_TYPE = 99,
} oem_board_type;

void OEM_board_judgement(void);

oem_board_type OEM_get_board(void);

#endif /* _KC_BOARD_H_ */
