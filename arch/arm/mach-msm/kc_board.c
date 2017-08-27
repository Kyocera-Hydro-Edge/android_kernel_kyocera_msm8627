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

/*==========================================================================

                     INCLUDE FILES FOR MODULE

==========================================================================*/

#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/module.h>
#include <mach/gpio.h>
#include <mach/kc_board.h>

/*------------------------------------------------------------------------
  Local define
  ------------------------------------------------------------------------*/
#define     GPIO_MASK_138           0x00000400L
#define     GPIO_MASK_144           0x00000800L
#define     GPIO_MASK_145           0x00001000L

#define     OEM_PULLDOWN            0x00000001
#define     OEM_NONEPULL            0x00000000

#define     HIGH_VALUE              0x01
#define     LOW_VALUE               0x00

#define     BOARD_CHECK1_NUM        138
#define     BOARD_CHECK2_NUM        144
#define     BOARD_CHECK3_NUM        145

#define     PORT_MASK_LLL           0x00
#define     PORT_MASK_LLH           0x01
#define     PORT_MASK_LHL           0x02
#define     PORT_MASK_LHH           0x03
#define     PORT_MASK_HLL           0x04
#define     PORT_MASK_HLH           0x05
#define     PORT_MASK_HHL           0x06
#define     PORT_MASK_HHH           0x07

#define BOARD_CHECK1_NO_PULL \
   GPIO_CFG(BOARD_CHECK1_NUM, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define BOARD_CHECK2_NO_PULL \
   GPIO_CFG(BOARD_CHECK2_NUM, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define BOARD_CHECK3_NO_PULL \
   GPIO_CFG(BOARD_CHECK3_NUM, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)

/*------------------------------------------------------------------------
  Local Structure 
  ------------------------------------------------------------------------*/

static uint32_t kc_board_cfg[] = {
   GPIO_CFG(BOARD_CHECK1_NUM, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
   GPIO_CFG(BOARD_CHECK2_NUM, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
   GPIO_CFG(BOARD_CHECK3_NUM, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

/*------------------------------------------------------------------------
  Local Variable
  ------------------------------------------------------------------------*/
static oem_board_type _board_type = OEM_BOARD_PP_TYPE;

/*===========================================================================

  FUNCTION OEM_board_judgement()

  DESCRIPTION
    Judge board type.

  PARAMETERS
    None.

  RETURN VALUE
    None.

  SIDE EFFECTS
    None.

===========================================================================*/
void OEM_board_judgement(void)
{
  int _value_check1,_value_check2,_value_check3=LOW_VALUE;
  int num, result=0;
  uint8_t               _port_mask=0;
  
  /* Initialize CHECK1 and CHECK2 and CHECK3 */
  result = gpio_request( BOARD_CHECK1_NUM, "board_check1" );
  
  if( result != 0 )
  {
     printk( KERN_ERR "OEM_board_judgement() : gpio_request() Failed PORT=%d",
                                                             BOARD_CHECK1_NUM );
  }
  
  result = gpio_request( BOARD_CHECK2_NUM, "board_check2" );
  
  if( result != 0 )
  {
     printk( KERN_ERR "OEM_board_judgement() : gpio_request() Failed PORT=%d",
                                                             BOARD_CHECK2_NUM );
  }
  
  result = gpio_request( BOARD_CHECK3_NUM, "board_check3" );
  
  if( result != 0 )
  {
     printk( KERN_ERR "OEM_board_judgement() : gpio_request() Failed PORT=%d",
                                                             BOARD_CHECK3_NUM );
  }
  
  /* Set Config for CHECK1 and CHECK2 and CHECK3 */
  for( num=0 ; num < ARRAY_SIZE(kc_board_cfg); num++ )
  {
     result = gpio_tlmm_config( kc_board_cfg[num], GPIO_CFG_ENABLE);
     
     if( result != 0 )
     {
        printk( KERN_ERR "OEM_board_judgement() : Initialized num=%d Error", num );
     }
  }
  
  /* Read CHECK1, CHECK2, CHECK3 */
  _value_check1 = gpio_get_value( BOARD_CHECK1_NUM );
  _value_check2 = gpio_get_value( BOARD_CHECK2_NUM );
  _value_check3 = gpio_get_value( BOARD_CHECK3_NUM );

  /* Port Mask Set */
  _port_mask = ((uint8_t)_value_check1 | ((uint8_t)_value_check2 << 1 ) | ((uint8_t)_value_check3 << 2 ));

  /* Board Type Judge */
  switch( _port_mask ){
    /* check1=Low check2=Low check3=Low */
    case PORT_MASK_LLL:
       /* For PP */
        _board_type = OEM_BOARD_PP_TYPE;
       /* No Process */
       break;
    /* check1=High check2=Low check3=Low */
    case PORT_MASK_LLH:
       /* For WS1 */
        _board_type = OEM_BOARD_WS1_TYPE;
       /* Set check1 Non-Pull */
       gpio_tlmm_config( BOARD_CHECK1_NO_PULL, GPIO_CFG_ENABLE );
       break;
    /* check1=Low check2=High check3=Low */
    case PORT_MASK_LHL:
       /* For WS2 */
        _board_type = OEM_BOARD_WS2_TYPE;
       /* Set check2 Non-Pull */
       gpio_tlmm_config( BOARD_CHECK2_NO_PULL, GPIO_CFG_ENABLE );
       break;
    /* check1=High check2=High check3=Low */
    case PORT_MASK_LHH:
       /* Set check1 Non-Pull */
       gpio_tlmm_config( BOARD_CHECK1_NO_PULL, GPIO_CFG_ENABLE );
       /* Set check2 Non-Pull */
       gpio_tlmm_config( BOARD_CHECK2_NO_PULL, GPIO_CFG_ENABLE );
       break;
    /* check1=Low check2=Low check3=High */
    case PORT_MASK_HLL:
       /* For LAB */
        _board_type = OEM_BOARD_LAB_TYPE;
       /* Set check3 Non-Pull */
       gpio_tlmm_config( BOARD_CHECK3_NO_PULL, GPIO_CFG_ENABLE );
       break;
    /* check1=High check2=Low check3=High */
    case PORT_MASK_HLH:
       /* For UT */
        _board_type = OEM_BOARD_UT_TYPE;
       /* Set check1 Non-Pull */
       gpio_tlmm_config( BOARD_CHECK1_NO_PULL, GPIO_CFG_ENABLE );
       /* Set check3 Non-Pull */
       gpio_tlmm_config( BOARD_CHECK3_NO_PULL, GPIO_CFG_ENABLE );
       break;
    /* check1=Low check2=High check3=High */
    case PORT_MASK_HHL:
       /* Set check2 Non-Pull */
       gpio_tlmm_config( BOARD_CHECK2_NO_PULL, GPIO_CFG_ENABLE );
       /* Set check3 Non-Pull */
       gpio_tlmm_config( BOARD_CHECK3_NO_PULL, GPIO_CFG_ENABLE );
       break;
    /* check1=High check2=High check3=High */
    case PORT_MASK_HHH:
       /* For WS0 */
        _board_type = OEM_BOARD_WS0_TYPE;
       /* Set check1 Non-Pull */
       gpio_tlmm_config( BOARD_CHECK1_NO_PULL, GPIO_CFG_ENABLE );
       /* Set check2 Non-Pull */
       gpio_tlmm_config( BOARD_CHECK2_NO_PULL, GPIO_CFG_ENABLE );
       /* Set check3 Non-Pull */
       gpio_tlmm_config( BOARD_CHECK3_NO_PULL, GPIO_CFG_ENABLE ); 
       break;
    default:
       break;
  }
}

/*===========================================================================

  FUNCTION OEM_get_board()

  DESCRIPTION
    get type

  PARAMETERS
    None.

  RETURN VALUE
    oem_board_type : Board Type

  SIDE EFFECTS
    None.

===========================================================================*/
oem_board_type OEM_get_board(void)
{
  return( _board_type );
}
EXPORT_SYMBOL(OEM_get_board);

