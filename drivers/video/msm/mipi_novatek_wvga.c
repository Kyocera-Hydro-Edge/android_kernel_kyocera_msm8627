/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */

#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_novatek_wvga.h"

#include <linux/wakelock.h>

extern void dev_lcd_force_mipi_clk_hs( int onoff );

struct wake_lock dev_lcd_wakelock;
struct wake_lock dev_lcd_wakelock2;

#define	DEV_LCD_WAKE_LOCK_INIT()	{}
#define	DEV_LCD_WAKE_LOCK()			{}
#define	DEV_LCD_WAKE_UNLOCK()		{}

int lcd_bl_on_flg = FALSE;
int bl_que_flag = 1;
int lcd_off_charge_flg = FALSE;

static struct mipi_dsi_panel_platform_data *mipi_novatek_wvga_pdata;


static struct dsi_buf novatek_wvga_tx_buf;
static struct dsi_buf novatek_wvga_rx_buf;
static int mipi_novatek_wvga_lcd_init(void);

extern void dev_lcd_force_mipi_clk_hs( int onoff );
int lcd_panel_judge_exec(int NumGPIO);
int do_lcd_panel_judge_exec(int NumGPIO,int num);
void lcd_panel_judge_sequece(void);



#define DEV_LCD_GPIO_RST    48
#define DEV_LCD_GPIO_BL_EN   1
#define DEV_LCD_GPIO_DET_ZERO    99
#define DEV_LCD_GPIO_DET_FIRST 146
#define DEV_LCD_GPIO_DET_SECOND 147

#define DEV_LCD_RST_INIT() \
  gpio_request( DEV_LCD_GPIO_RST, "lcd_rst_init"); \
  gpio_tlmm_config(GPIO_CFG( DEV_LCD_GPIO_RST , 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE); \
  gpio_free(DEV_LCD_GPIO_RST)

#define DEV_LCD_RST_HI() \
  gpio_request( DEV_LCD_GPIO_RST, "lcd_rst_hi"); \
  gpio_direction_output(DEV_LCD_GPIO_RST , 1); \
  gpio_free(DEV_LCD_GPIO_RST)

#define DEV_LCD_RST_LO() \
  gpio_request( DEV_LCD_GPIO_RST, "lcd_rst_lo"); \
  gpio_direction_output(DEV_LCD_GPIO_RST , 0); \
  gpio_free(DEV_LCD_GPIO_RST)

#define DEV_LCD_BL_INIT()
#define DEV_LCD_BL_ENA() 
#define DEV_LCD_BL_DIS() 

#define DEV_LCD_GPIO_SET( gpio, pull, io ) \
  gpio_request( gpio, "lcd_gpio"); \
  gpio_tlmm_config(GPIO_CFG( gpio , 0, io , pull, GPIO_CFG_2MA), GPIO_CFG_ENABLE); \
  gpio_free(gpio); \
  msleep(1)

#define DEV_LCD_GPIO_READ(gpio, hilo) \
do{ \
  gpio_request( gpio, "lcd_read");  \
  gpio_direction_input(gpio); \
  msleep(1);   \
  hilo = gpio_get_value(gpio); \
  gpio_free(gpio);                  \
  msleep(1); \
}while(0)

#define DEV_LCD_GPIO_LO(gpio) \
  gpio_request(gpio, "gpio_lo"); \
  gpio_direction_output(gpio , 0); \
  gpio_free(gpio); \
  msleep(1)

#define LCD_PANEL_SELECT1 1
#define LCD_PANEL_SELECT2 2
int dev_lcd_panel_select_flg = LCD_PANEL_SELECT1;

#define LCDBL_GPIO_PULSE 6
void lcdbl_gpio_pulse( int pulse )
{
  int i;
  for( i = 0; i < pulse; i++)
  {
    DEV_LCD_BL_DIS();
    udelay(10);
    DEV_LCD_BL_ENA();
    udelay(10);
  }
}

#define	LCDC_PARAMETER_MAX	(64)
#define	LCDC_TYPE_END	-1
#define	LCDC_TYPE_NONE	-2
#define	LCDC_TYPE_RESET	-3
#define	LCDC_PARAM_USER	-1

struct	mipi_lcdc_sequence{
	int	iType;
	int	iCommand;
	int	iSize;
	int	iParameter[LCDC_PARAMETER_MAX];
	int	iWait;
};

static struct mipi_lcdc_sequence	mipi_LG4573B_sequence_start1[]={
	{LCDC_TYPE_RESET,1,0,{},10},
	{LCDC_TYPE_RESET,0,0,{},10},
	{LCDC_TYPE_RESET,1,0,{},120},
	{0x39,0xF0, 5,{0x55,0xAA,0x52,0x08,0x01},0},
	{0x39,0xB0, 3,{0x03,0x03,0x03},0},
	{0x39,0xB6, 3,{0x46,0x46,0x46},0},
	{0x39,0xB1, 3,{0x03,0x03,0x03},0},
	{0x39,0xB7, 3,{0x36,0x36,0x36},0},
	{0x39,0xB2, 3,{0x00,0x00,0x00},0},
	{0x39,0xB8, 3,{0x26,0x26,0x26},0},
	{0x39,0xB3, 3,{0x09,0x09,0x09},0},
	{0x39,0xB9, 3,{0x36,0x36,0x36},0},
	{0x39,0xB5, 3,{0x08,0x08,0x08},0},
	{0x39,0xBA, 3,{0x26,0x26,0x26},0},
	{0x39,0xBC, 3,{0x00,0x80,0x00},0},
	{0x39,0xBD, 3,{0x00,0x80,0x00},0},
	{0x39,0xBE, 2,{0x00,0x73},0},
	{0x39,0xD1,52,{0x00,0x2D,0x00,0x41,0x00,0x60,0x00,0x79,0x00,0x90,0x00,0xB9,0x00,0xD9,0x01,0x0B,0x01,0x38,0x01,0x7A,0x01,0xAD,0x01,0xFB,0x02,0x3C,0x02,0x3E,0x02,0x77,0x02,0xB2,0x02,0xD4,0x02,0xFE,0x03,0x1A,0x03,0x3E,0x03,0x52,0x03,0x68,0x03,0x71,0x03,0x7A,0x03,0x7E,0x03,0x7F},0},
	{0x39,0xD2,52,{0x00,0x2D,0x00,0x41,0x00,0x60,0x00,0x79,0x00,0x90,0x00,0xB9,0x00,0xD9,0x01,0x0B,0x01,0x38,0x01,0x7A,0x01,0xAD,0x01,0xFB,0x02,0x3C,0x02,0x3E,0x02,0x77,0x02,0xB2,0x02,0xD4,0x02,0xFE,0x03,0x1A,0x03,0x3E,0x03,0x52,0x03,0x68,0x03,0x71,0x03,0x7A,0x03,0x7E,0x03,0x7F},0},
	{0x39,0xD3,52,{0x00,0x2D,0x00,0x41,0x00,0x60,0x00,0x79,0x00,0x90,0x00,0xB9,0x00,0xD9,0x01,0x0B,0x01,0x38,0x01,0x7A,0x01,0xAD,0x01,0xFB,0x02,0x3C,0x02,0x3E,0x02,0x77,0x02,0xB2,0x02,0xD4,0x02,0xFE,0x03,0x1A,0x03,0x3E,0x03,0x52,0x03,0x68,0x03,0x71,0x03,0x7A,0x03,0x7E,0x03,0x7F},0},
	{0x39,0xD4,52,{0x00,0x2D,0x00,0x41,0x00,0x60,0x00,0x79,0x00,0x90,0x00,0xB9,0x00,0xD9,0x01,0x0B,0x01,0x38,0x01,0x7A,0x01,0xAD,0x01,0xFB,0x02,0x3C,0x02,0x3E,0x02,0x77,0x02,0xB2,0x02,0xD4,0x02,0xFE,0x03,0x1A,0x03,0x3E,0x03,0x52,0x03,0x68,0x03,0x71,0x03,0x7A,0x03,0x7E,0x03,0x7F},0},
	{0x39,0xD5,52,{0x00,0x2D,0x00,0x41,0x00,0x60,0x00,0x79,0x00,0x90,0x00,0xB9,0x00,0xD9,0x01,0x0B,0x01,0x38,0x01,0x7A,0x01,0xAD,0x01,0xFB,0x02,0x3C,0x02,0x3E,0x02,0x77,0x02,0xB2,0x02,0xD4,0x02,0xFE,0x03,0x1A,0x03,0x3E,0x03,0x52,0x03,0x68,0x03,0x71,0x03,0x7A,0x03,0x7E,0x03,0x7F},0},
	{0x39,0xD6,52,{0x00,0x2D,0x00,0x41,0x00,0x60,0x00,0x79,0x00,0x90,0x00,0xB9,0x00,0xD9,0x01,0x0B,0x01,0x38,0x01,0x7A,0x01,0xAD,0x01,0xFB,0x02,0x3C,0x02,0x3E,0x02,0x77,0x02,0xB2,0x02,0xD4,0x02,0xFE,0x03,0x1A,0x03,0x3E,0x03,0x52,0x03,0x68,0x03,0x71,0x03,0x7A,0x03,0x7E,0x03,0x7F},0},
	{0x39,0xF0, 5,{0x55,0xAA,0x52,0x08,0x00},0},
	{0x39,0xB1, 2,{0xFC,0x06},0},
	{0x15,0xB6, 1,{0x0A},0},
	{0x39,0xB7, 2,{0x00,0x00},0},
	{0x39,0xB8, 4,{0x01,0x05,0x05,0x05},0},
	{0x39,0xBC, 3,{0x00,0x00,0x00},0},
	{0x39,0xCC, 3,{0x03,0x00,0x00},0},
	{0x15,0xBA, 1,{0x01},0},
	{0x15,0x35, 1,{0x00},0},
	{0x15,0x3A, 1,{0x77},0},
	{0x15,0x53,1,{0x24},0},
	{0x15,0x55,1,{0x00},0},
	{0x15,0x5E,1,{0x10},0},
	{0x05,0x11,0,{},120},
	{0x05,0x29,0,{},0},
	{LCDC_TYPE_END,-1,-1,{},-1}
};

static struct mipi_lcdc_sequence	mipi_LG4573B_sequence_start2[]={
	{LCDC_TYPE_RESET,1,0,{},10},
	{LCDC_TYPE_RESET,0,0,{},10},
	{LCDC_TYPE_RESET,1,0,{},120},
	{0x39,0xF0, 5,{0x55,0xAA,0x52,0x08,0x01},0},
	{0x39,0xB0, 3,{0x03,0x03,0x03},0},
	{0x39,0xB6, 3,{0x46,0x46,0x46},0},
	{0x39,0xB1, 3,{0x03,0x03,0x03},0},
	{0x39,0xB7, 3,{0x36,0x36,0x36},0},
	{0x39,0xB2, 3,{0x00,0x00,0x00},0},
	{0x39,0xB8, 3,{0x26,0x26,0x26},0},
	{0x39,0xB3, 3,{0x09,0x09,0x09},0},
	{0x39,0xB9, 3,{0x36,0x36,0x36},0},
	{0x39,0xB5, 3,{0x08,0x08,0x08},0},
	{0x39,0xBA, 3,{0x26,0x26,0x26},0},
	{0x39,0xBC, 3,{0x00,0x80,0x00},0},
	{0x39,0xBD, 3,{0x00,0x80,0x00},0},
	{0x39,0xBE, 2,{0x00,0x73},0},
	{0x39,0xD1,52,{0x00,0x2D,0x00,0x45,0x00,0x66,0x00,0x7F,0x00,0x94,0x00,0xB4,0x00,0xD0,0x00,0xFE,0x01,0x22,0x01,0x59,0x01,0x87,0x01,0xD1,0x02,0x0C,0x02,0x0E,0x02,0x45,0x02,0x7E,0x02,0xA4,0x02,0xD7,0x02,0xFA,0x03,0x28,0x03,0x45,0x03,0x61,0x03,0x6E,0x03,0x78,0x03,0x7E,0x03,0x7F},0},
	{0x39,0xD2,52,{0x00,0x2D,0x00,0x45,0x00,0x66,0x00,0x7F,0x00,0x94,0x00,0xB4,0x00,0xD0,0x00,0xFE,0x01,0x22,0x01,0x59,0x01,0x87,0x01,0xD1,0x02,0x0C,0x02,0x0E,0x02,0x45,0x02,0x7E,0x02,0xA4,0x02,0xD7,0x02,0xFA,0x03,0x28,0x03,0x45,0x03,0x61,0x03,0x6E,0x03,0x78,0x03,0x7E,0x03,0x7F},0},
	{0x39,0xD3,52,{0x00,0x2D,0x00,0x45,0x00,0x66,0x00,0x7F,0x00,0x94,0x00,0xB4,0x00,0xD0,0x00,0xFE,0x01,0x22,0x01,0x59,0x01,0x87,0x01,0xD1,0x02,0x0C,0x02,0x0E,0x02,0x45,0x02,0x7E,0x02,0xA4,0x02,0xD7,0x02,0xFA,0x03,0x28,0x03,0x45,0x03,0x61,0x03,0x6E,0x03,0x78,0x03,0x7E,0x03,0x7F},0},
	{0x39,0xD4,52,{0x00,0x2D,0x00,0x45,0x00,0x66,0x00,0x7F,0x00,0x94,0x00,0xB4,0x00,0xD0,0x00,0xFE,0x01,0x22,0x01,0x59,0x01,0x87,0x01,0xD1,0x02,0x0C,0x02,0x0E,0x02,0x45,0x02,0x7E,0x02,0xA4,0x02,0xD7,0x02,0xFA,0x03,0x28,0x03,0x45,0x03,0x61,0x03,0x6E,0x03,0x78,0x03,0x7E,0x03,0x7F},0},
	{0x39,0xD5,52,{0x00,0x2D,0x00,0x45,0x00,0x66,0x00,0x7F,0x00,0x94,0x00,0xB4,0x00,0xD0,0x00,0xFE,0x01,0x22,0x01,0x59,0x01,0x87,0x01,0xD1,0x02,0x0C,0x02,0x0E,0x02,0x45,0x02,0x7E,0x02,0xA4,0x02,0xD7,0x02,0xFA,0x03,0x28,0x03,0x45,0x03,0x61,0x03,0x6E,0x03,0x78,0x03,0x7E,0x03,0x7F},0},
	{0x39,0xD6,52,{0x00,0x2D,0x00,0x45,0x00,0x66,0x00,0x7F,0x00,0x94,0x00,0xB4,0x00,0xD0,0x00,0xFE,0x01,0x22,0x01,0x59,0x01,0x87,0x01,0xD1,0x02,0x0C,0x02,0x0E,0x02,0x45,0x02,0x7E,0x02,0xA4,0x02,0xD7,0x02,0xFA,0x03,0x28,0x03,0x45,0x03,0x61,0x03,0x6E,0x03,0x78,0x03,0x7E,0x03,0x7F},0},
	{0x39,0xF0, 5,{0x55,0xAA,0x52,0x08,0x00},0},
	{0x39,0xB1, 2,{0xFC,0x06},0},
	{0x15,0xB6, 1,{0x0A},0},
	{0x39,0xB7, 2,{0x00,0x00},0},
	{0x39,0xB8, 4,{0x01,0x05,0x05,0x05},0},
	{0x39,0xBC, 3,{0x00,0x00,0x00},0},
	{0x39,0xCC, 3,{0x03,0x00,0x00},0},
	{0x15,0xBA, 1,{0x01},0},
	{0x15,0x35, 1,{0x00},0},
	{0x15,0x3A, 1,{0x77},0},
	{0x15,0x53,1,{0x24},0},
	{0x15,0x55,1,{0x00},0},
	{0x15,0x5E,1,{0x10},0},
	{0x05,0x11,0,{},120},
	{0x05,0x29,0,{},0},
	{LCDC_TYPE_END,-1,-1,{},-1}
};

static struct mipi_lcdc_sequence	mipi_LG4573B_sequence_deep_standby[]={
	{0x05,0x28,0,{},0},
	{0x05,0x10,0,{},200},
	{LCDC_TYPE_END,-1,-1,{},-1}
};

static struct mipi_lcdc_sequence	mipi_LG4573B_sequence_deep_standby2[]={
	{0x15,0x4F,1,{0x01},0},
	{LCDC_TYPE_END,-1,-1,{},-1}
};

static struct mipi_lcdc_sequence	mipi_LG4573B_sequence_cabc_setting[]={
	{0x15,0x55,1,{LCDC_PARAM_USER},0},
	{LCDC_TYPE_END,-1,-1,{},-1}
};


#define	mipi_LG4573B_deep_resume	mipi_LG4573B_sequence_start

#define DI05       DTYPE_DCS_WRITE
#define DI15       DTYPE_DCS_WRITE1
#define DI23       DTYPE_GEN_WRITE2
#define DI29       DTYPE_GEN_LWRITE
#define DI39       DTYPE_DCS_LWRITE

void dev_lcd_mipi_cmd_send( int dtype, int dlen, char *parameter, struct msm_fb_data_type *mfd)
{
  struct dsi_cmd_desc cmds = {0};
  cmds.dtype   = dtype;
  cmds.last    = 1;
  cmds.vc      = 0;
  cmds.ack     = 0;
  cmds.wait    = 0;
  cmds.dlen    = dlen;
  cmds.payload = parameter;

  mipi_dsi_cmds_tx(&novatek_wvga_tx_buf, &cmds , 1/* cnt */);
}

void mipi_sequence_exec( struct mipi_lcdc_sequence *slsSequence , char *szptUser , struct msm_fb_data_type *mfd)
{
int iLoop;
char szParameter[LCDC_PARAMETER_MAX+1] = {0};

	while(slsSequence->iType != LCDC_TYPE_END){
		if(slsSequence->iType == LCDC_TYPE_NONE){
		}
		else if(slsSequence->iType == LCDC_TYPE_RESET){

			if(slsSequence->iCommand == 0){	DEV_LCD_RST_LO();	}
			else{	DEV_LCD_RST_HI();	}
		}
		else{
			szParameter[0] = slsSequence->iCommand;
			for(iLoop=0 ; iLoop < slsSequence->iSize ; iLoop++){
				if(slsSequence->iParameter[iLoop] == LCDC_PARAM_USER){
					if(szptUser == (char *)NULL){	szParameter[iLoop+1] = 0x00;	}
					else{	szParameter[iLoop+1] = *szptUser;	szptUser++;	}
				}
				else{	szParameter[iLoop+1] = slsSequence->iParameter[iLoop];	}
			}
			dev_lcd_mipi_cmd_send(slsSequence->iType,(slsSequence->iSize)+1, (char *)szParameter, mfd);
		}
		if(slsSequence->iWait > 0){	msleep(slsSequence->iWait);	}
		slsSequence++;
	}
}

static bool mipi_novatek_wvga_panel_power_on = false;
static int  mipi_novatek_wvga_panel_power_status = -1;
static struct regulator *reg_l15, *reg_l2;
DEFINE_MUTEX(mipi_novatek_wvga_panel_lock);
DEFINE_MUTEX(mipi_novatek_wvga_lcd_disp);

static int mipi_novatek_wvga_panel_power( int on )
{
	int rc;
	static int first_flg = 0;
	printk("%s start:state=%d req=%d\n", __func__,mipi_novatek_wvga_panel_power_status, on);

	if(mipi_novatek_wvga_panel_power_status == on){
		return 0;
	}

	mutex_lock(&mipi_novatek_wvga_panel_lock);
	if (!mipi_novatek_wvga_panel_power_on)
	{
		reg_l15 = regulator_get(NULL, "8038_l15");
		if (IS_ERR(reg_l15)) {
			pr_err("could not get pm8038_l15, rc = %ld\n", PTR_ERR(reg_l15));
			goto panel_power_err;
		}
		reg_l2 = regulator_get(NULL, "8038_l2");
		if (IS_ERR(reg_l2)) {
			pr_err("could not get 8038_l2, rc = %ld\n", PTR_ERR(reg_l2));
			goto panel_power_err;
		}
		rc = regulator_set_voltage(reg_l15, 2800000, 2800000);
		if (rc) {
			pr_err("set_voltage l15 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
		rc = regulator_set_voltage(reg_l2, 1200000, 1200000);
		if (rc) {
			pr_err("set_voltage l2 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
		
		rc = regulator_set_optimum_mode(reg_l15, 300000);
		if (rc < 0) {
			pr_err(KERN_ERR "set_optimum_mode l15 failed, rc=%d\n", rc);
			goto panel_power_err;
		}		

		rc = regulator_enable(reg_l15);
		if (rc) {
			pr_err("enable l8 failed, rc=%d\n", rc);
			goto panel_power_err;
		}		

		rc = regulator_disable(reg_l15);
		if (rc) {
			pr_err("disable reg_l15 failed, rc=%d\n", rc);
			goto panel_power_err;
		}

		rc = regulator_set_optimum_mode(reg_l15, 300000);
		if (rc < 0) {
			pr_err(KERN_ERR "set_optimum_mode l15 failed, rc=%d\n", rc);
			goto panel_power_err;
		}

		rc = regulator_enable(reg_l15);
		if (rc) {
			pr_err("enable l15 failed, rc=%d\n", rc);
			goto panel_power_err;
		}		
		
		mipi_novatek_wvga_panel_power_on = true;
	}

	if (on)
	{
		rc = regulator_set_optimum_mode(reg_l2, 100000);
		if (rc < 0) {
			pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
			goto panel_power_err;
		}

		rc = regulator_enable(reg_l2);
		if (rc) {
			pr_err("enable l2 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
	  if(first_flg == 0){
		rc = regulator_set_optimum_mode(reg_l15, 300000);
		if (rc < 0) {
			pr_err(KERN_ERR "set_optimum_mode l15 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
		rc = regulator_enable(reg_l15);
		if (rc) {
			pr_err("enable l15 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
		first_flg = 1;
	  }
	}
	else
	{
		rc = regulator_disable(reg_l2);
		if (rc) {
			pr_err("disable reg_l2 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
		rc = regulator_set_optimum_mode(reg_l2, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
			goto panel_power_err;
		}
	}
	mipi_novatek_wvga_panel_power_status = on;
	mutex_unlock(&mipi_novatek_wvga_panel_lock);
	printk("%s:end\n", __func__);

	return 0;

panel_power_err:
	mutex_unlock(&mipi_novatek_wvga_panel_lock);
	printk("%s:err end\n", __func__);
	return -ENODEV;
}

int dev_lcd_panel_power_ctrl( int onoff )
{
  return(mipi_novatek_wvga_panel_power(onoff));
}

#define LCD_POWER28_OFF_TIME 100
void mipi_lcd_28v_off_temporary( void )
{
	int rc;
	rc = regulator_disable(reg_l15);
	if (rc) {
		pr_err("enable l15 failed, rc=%d\n", rc);
	}
	msleep(LCD_POWER28_OFF_TIME);
	rc = regulator_set_optimum_mode(reg_l15, 300000);
	if (rc < 0) {
		pr_err(KERN_ERR "set_optimum_mode l15 failed, rc=%d\n", rc);
	}
	rc = regulator_enable(reg_l15);
	if (rc) {
		pr_err("enable l15 failed, rc=%d\n", rc);
	}
}

int mipi_novatek_wvga_lcd_on_exec(struct msm_fb_data_type *mfd)
{
	printk("[DEV LCD]%s:start\n", __func__);

	if(lcd_bl_on_flg){
		return 0;
	}
	lcd_bl_on_flg = TRUE;

	DEV_LCD_WAKE_LOCK();

	mutex_lock(&mipi_novatek_wvga_lcd_disp);

	msleep(10);

	mipi_set_tx_power_mode(1);

	if(dev_lcd_panel_select_flg == LCD_PANEL_SELECT2){
		mipi_sequence_exec(mipi_LG4573B_sequence_start2, (char*)NULL, mfd);
	}else{
		mipi_sequence_exec(mipi_LG4573B_sequence_start1, (char*)NULL, mfd);
	}

	mipi_set_tx_power_mode(0);

	mutex_unlock(&mipi_novatek_wvga_lcd_disp);
	DEV_LCD_WAKE_UNLOCK();
	printk("[DEV LCD]%s:end\n", __func__);
	return 0;
}

static int mipi_novatek_wvga_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	return mipi_novatek_wvga_lcd_on_exec(mfd);
}

int mipi_novatek_wvga_lcd_off_exec(struct msm_fb_data_type *mfd)
{
	printk("[DEV LCD]%s:start\n", __func__);

	oem_lcd_backlight_ctrl(0);
	if(!lcd_bl_on_flg){
		return 0;
	}
	lcd_bl_on_flg = FALSE;
	bl_que_flag = 0;

	DEV_LCD_WAKE_LOCK();
	mutex_lock(&mipi_novatek_wvga_lcd_disp);

	DEV_LCD_BL_DIS();
	msleep(10);
	mipi_set_tx_power_mode(1);

	mipi_sequence_exec(mipi_LG4573B_sequence_deep_standby, (char*)NULL, mfd);

	DEV_LCD_RST_LO();
	msleep(100);
	DEV_LCD_RST_HI();
	msleep(200);

	mipi_sequence_exec(mipi_LG4573B_sequence_deep_standby2, (char*)NULL, mfd);

	mutex_unlock(&mipi_novatek_wvga_lcd_disp);
	DEV_LCD_WAKE_UNLOCK();
	printk("[DEV LCD]%s:end\n", __func__);
	return 0;
}

static int mipi_novatek_wvga_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	return mipi_novatek_wvga_lcd_off_exec(mfd);
}

int dev_lcd_display_on( struct msm_fb_data_type *mfd )
{
  return mipi_novatek_wvga_lcd_on_exec(mfd);
}

int dev_lcd_display_off( struct msm_fb_data_type *mfd )
{
  return mipi_novatek_wvga_lcd_off_exec(mfd);
}

struct msm_fb_data_type * mipi_novatek_wvga_get_mfd(void)
{
	struct platform_device *pdev = NULL;
	struct msm_fb_data_type *mfd;

	pdev = platform_device_alloc("mipi_novatek_wvga", (MIPI_DSI_PANEL_WVGA << 8)|MIPI_DSI_PRIM);
	mfd = platform_get_drvdata(pdev);

	return mfd;
}

int dev_lcdbl_hw_pwm_calc( int and )
{
  static int pre_and = 0;
  int pwm;
  if(and > 255){
    and = 255;
  }else if(and <= 0){
    and = 0;
	pwm = and;
  }
  if(dev_lcd_panel_select_flg == LCD_PANEL_SELECT2){
    if(and != 0){
      pwm = (204 * and) / 255;
	}
  }else{
    if(and != 0){
      pwm = (204 * and) / 255;
    }
  }

  if((abs(and - pre_and) >= 15) || (and == 0))
  {
    printk("%s()AND[%d] HW[%d] \n", __func__, and, pwm);
    pre_and = and;
  }
  return(pwm);
}

void mipi_novatek_wvga_set_cabc_ctrl(int onoff)
{
  char  szBuffer[1];
  struct msm_fb_data_type *mfd;
  mfd = mipi_novatek_wvga_get_mfd();
  if(onoff){
    szBuffer[0] = 0x03;
  }else{
    szBuffer[0] = 0x00;
  }
  printk("[DEV LCD] CABC SET(%x) \n", szBuffer[0]);
  mipi_sequence_exec(mipi_LG4573B_sequence_cabc_setting, szBuffer, mfd);

}

static int __devinit mipi_novatek_wvga_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mipi_novatek_wvga_pdata = pdev->dev.platform_data;
		return 0;
	}

	if (mipi_novatek_wvga_pdata == NULL) {
		pr_err("%s.invalid platform data.\n", __func__);
		return -ENODEV;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_novatek_wvga_lcd_probe,
	.driver = {
		.name   = "mipi_novatek_wvga",
	},
};

static struct msm_fb_panel_data novatek_wvga_panel_data = {
  .on            = mipi_novatek_wvga_lcd_on,
  .off           = mipi_novatek_wvga_lcd_off,
  .set_backlight = NULL,
};

static int ch_used[3];
int mipi_novatek_wvga_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

    printk("%s() \n", __func__);

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	ret = mipi_novatek_wvga_lcd_init();
	if (ret) {
		pr_err("mipi_novatek_wvga_lcd_init() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_novatek_wvga", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	novatek_wvga_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &novatek_wvga_panel_data,
		sizeof(novatek_wvga_panel_data));
	if (ret) {
		pr_err(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int mipi_novatek_wvga_lcd_init(void)
{
  printk("%s() \n", __func__);

  mipi_dsi_buf_alloc(&novatek_wvga_tx_buf, DSI_BUF_SIZE);
  mipi_dsi_buf_alloc(&novatek_wvga_rx_buf, DSI_BUF_SIZE);

  DEV_LCD_RST_INIT();
  DEV_LCD_BL_INIT();
  DEV_LCD_WAKE_LOCK_INIT();

  lcd_panel_judge_sequece();
  
  mipi_novatek_wvga_panel_power(1);
  lcd_bl_on_flg = FALSE;
  if(oem_lcd_off_charge_check()){
    lcd_off_charge_flg = TRUE;
  }

  return platform_driver_register(&this_driver);
}

#define MAIN_LCD_START_X                        0
#define MAIN_LCD_START_Y                        0
#define MAIN_LCD_WIDTH                          480
#define MAIN_LCD_HIGHT                          800
#define MAIN_LCD_END_X                          (MAIN_LCD_START_X+MAIN_LCD_WIDTH)
#define MAIN_LCD_END_Y                          (MAIN_LCD_START_Y+MAIN_LCD_HIGHT)

#define SR_ID_POS 0
#define SR_BIT_DEPTH_POS 2
#define SR_TRANS_FLG_POS 3
#define SR_BYTE_PER_DOT_POS 4
#define SR_PACK_FLG_POS 5
#define SR_IMAGE_WIDTH_POS 6
#define SR_IMAGE_HEIGHT_POS 8
#define SR_TP_COLOR_POS 12
#define SR_PALET_NUM_POS 13
#define SR_TRANS_INDEX_POS 14
#define SR_NO_TRANS 0
#define SR_TRANS 1
#define SR_TRANS_ALPHA 2
#define SR_BYTE_ORDER_MASK 0x01
#define SR_LITTLE_ENDIAN 0x00
#define SR_BIG_ENDIAN 0x01
#define SR_PACK_DIRECTION_MASK 0x02
#define SR_RIGHT_PACK 0x00
#define SR_LEFT_PACK 0x02
#define SR_PACK_LENGTH_MASK 0x04

#define SR_HEADER_SIZE 16

#define BOOT_XSTA_LIMIT(x,left)  (((x) < (left))? (left-x) : (0))
#define BOOT_YPOS_LIMIT(y,top)   (((top) > (y))?  (top-y)  : (0))
#define BOOT_XEND_LIMIT(x,width,right)   (((x+width-1) > (right))?   (right-x)  : (width-1))
#define BOOT_YEND_LIMIT(y,height,bottom) (((y+height-1) > (bottom))? (bottom-y) : (height-1))
#define BOOT_YPOS_ADD(ypos,y,column_size,x) (((ypos) + (y)) * (column_size) + (x))

static uint16 boot_disp_work1[MAIN_LCD_WIDTH];
static uint16 boot_disp_work2[MAIN_LCD_WIDTH];

void rgb565_to_rgba8888(short rgb565, char *out_buf)
{
  char r,g,b;
  r = (rgb565 & 0xF800) >> 8;
  g = (rgb565 & 0x07E0) >> 3;
  b = (rgb565 & 0x001F) << 3;
  out_buf[0] = r;
  out_buf[1] = g;
  out_buf[2] = b;
  out_buf[3] = 0x00;
}

void  kernel_disp_raw_bitmap(int x,
                              int y,
                              int column_size,
                              int line_size,
                              const unsigned char *imageData,
                              char *out_dbuf)
{
  int width,height;
  int xsta;
  int xend;
  int ypos;
  int top,bottom,left,right;
  int ypos_add;
  int i;

  if(!imageData)return;

  top    = 0;
  left   = 0;
  bottom = line_size   - 1;
  right  = column_size - 1;

  if( !memcmp(imageData,"SI",2) )
  {
    uint16  *palette;
    uint16  *disp_buf;
    char   *output_dbuf = (char *)out_dbuf;

    uint16 dotParByte;
    int yend;
    int copy_size;
    int cnt;
    int image_work;
    int pltcnt;
    int image_ix, disp_buf_ix, ix;

    copy_size  = 0;
    dotParByte = imageData[SR_BYTE_PER_DOT_POS];

    pltcnt = imageData[SR_PALET_NUM_POS];
    pltcnt++;

    width  = (uint16)imageData[SR_IMAGE_WIDTH_POS]  + (uint16)imageData[SR_IMAGE_WIDTH_POS  + 1] * 256;
    height = (uint16)imageData[SR_IMAGE_HEIGHT_POS] + (uint16)imageData[SR_IMAGE_HEIGHT_POS + 1] * 256;

    palette  = (uint16 *)boot_disp_work1;
    memcpy(palette,&imageData[SR_HEADER_SIZE],pltcnt * 2);
    disp_buf = (uint16 *)boot_disp_work2;

    image_ix  = SR_HEADER_SIZE + pltcnt * 2;
    xsta      = BOOT_XSTA_LIMIT(x,left);
    xend      = BOOT_XEND_LIMIT(x,width,right);
    ypos      = BOOT_YPOS_LIMIT(y,top);
    yend      = BOOT_YEND_LIMIT(y,height,bottom);
    ypos_add  = BOOT_YPOS_ADD(ypos,y,column_size,x);
    copy_size = (xend-xsta+1)<<1;

    if(dotParByte == 2)
    {
      for( ;ypos <= yend;ypos++)
      {
        if(imageData[image_ix])
        {
          image_ix++;
          disp_buf_ix = 0;

          while(TRUE)
          {
            cnt = imageData[image_ix];
            image_ix++;
            image_work = (int)imageData[image_ix];

            for(ix = 0;ix < cnt;ix++)
            {
              disp_buf[disp_buf_ix + ix] = palette[image_work];
            }
            image_ix ++;
            disp_buf_ix += cnt;
            if(disp_buf_ix >= width)
            {
              break;
            }
          }
        }
        else
        {
          for(ix = 0;ix < width;ix++)
          {
            disp_buf[ix] = palette[(imageData[image_ix + ix + 1])];
          }
          image_ix = image_ix + width + 1;
        }
        for( i = 0; i < copy_size; i++)
        {
          rgb565_to_rgba8888(disp_buf[xsta + i], (char *)&output_dbuf[(ypos_add + xsta + i) * 4]);
        }
        ypos_add += column_size;
      }
    }
  }
}

#define LCD_DET_LO_LO 0
#define LCD_DET_LO_HI 1
#define LCD_DET_HI_LO 2
#define LCD_DET_HI_HI 3

int lcd_panel_judge_exec(int NumGPIO)
{
  int dev_lcd_connect_flg1 = 0;
  int dev_lcd_connect_flg2 = 0;

  DEV_LCD_GPIO_SET(NumGPIO,GPIO_CFG_PULL_UP,GPIO_CFG_INPUT);
  DEV_LCD_GPIO_READ(NumGPIO, dev_lcd_connect_flg1);
  DEV_LCD_GPIO_SET(NumGPIO,GPIO_CFG_PULL_DOWN,GPIO_CFG_INPUT);
  DEV_LCD_GPIO_READ(NumGPIO, dev_lcd_connect_flg2);
  dev_lcd_connect_flg1 &= 0x01;
  dev_lcd_connect_flg2 &= 0x01;
  printk("%s(JUDGE[%x]) \n", __func__, (dev_lcd_connect_flg1 << 1) | dev_lcd_connect_flg2);
  return((dev_lcd_connect_flg1 << 1) | dev_lcd_connect_flg2);
}

int do_lcd_panel_judge_exec(int NumGPIO,int num)
{
  int judge;
  int do_next_judge_flg = 0;

  judge = lcd_panel_judge_exec(NumGPIO);

  switch(judge){
    case LCD_DET_HI_LO:
	     if(num==0){
			DEV_LCD_GPIO_SET(DEV_LCD_GPIO_DET_ZERO,GPIO_CFG_NO_PULL,GPIO_CFG_OUTPUT);
			DEV_LCD_GPIO_LO(DEV_LCD_GPIO_DET_ZERO);			
			DEV_LCD_GPIO_SET(DEV_LCD_GPIO_DET_FIRST,GPIO_CFG_NO_PULL,GPIO_CFG_OUTPUT);
		    DEV_LCD_GPIO_LO(DEV_LCD_GPIO_DET_FIRST);
			DEV_LCD_GPIO_SET(DEV_LCD_GPIO_DET_SECOND,GPIO_CFG_NO_PULL,GPIO_CFG_OUTPUT);
		    DEV_LCD_GPIO_LO(DEV_LCD_GPIO_DET_SECOND);
			printk("[DEV LCD] NOT_CONNECT !!! \n");
		 }else if(num==1){
			DEV_LCD_GPIO_SET(DEV_LCD_GPIO_DET_FIRST,GPIO_CFG_NO_PULL,GPIO_CFG_OUTPUT);
		    DEV_LCD_GPIO_LO(DEV_LCD_GPIO_DET_FIRST);
			DEV_LCD_GPIO_SET(DEV_LCD_GPIO_DET_SECOND,GPIO_CFG_NO_PULL,GPIO_CFG_OUTPUT);
			DEV_LCD_GPIO_LO(DEV_LCD_GPIO_DET_SECOND);
			printk("[DEV LCD] BEIJIN2_PANEL!!! \n");
		}else if(num==2){
			DEV_LCD_GPIO_SET(DEV_LCD_GPIO_DET_SECOND,GPIO_CFG_NO_PULL,GPIO_CFG_OUTPUT);
			DEV_LCD_GPIO_LO(DEV_LCD_GPIO_DET_SECOND);
			dev_lcd_panel_select_flg = LCD_PANEL_SELECT2;
			printk("[DEV LCD] HEFEI_PANEL!!! \n");
		}
      break;
    case LCD_DET_LO_LO:
  			if(num==0){
				DEV_LCD_GPIO_SET(DEV_LCD_GPIO_DET_ZERO,GPIO_CFG_PULL_DOWN,GPIO_CFG_INPUT);
				do_next_judge_flg = 1;
				printk("[DEV LCD] LCD_Connect\n");
			}else if(num==1){
				DEV_LCD_GPIO_SET(DEV_LCD_GPIO_DET_FIRST,GPIO_CFG_PULL_DOWN,GPIO_CFG_INPUT);
				do_next_judge_flg = 1;
				printk("[DEV LCD] LCD_Connect2 \n");
			}else if(num==2){
				DEV_LCD_GPIO_SET(DEV_LCD_GPIO_DET_SECOND,GPIO_CFG_PULL_DOWN,GPIO_CFG_INPUT);
				printk("[DEV LCD]BEIJIN1_PANEL!!! \n");
			}
      break;
    case LCD_DET_HI_HI:
	case LCD_DET_LO_HI:
			DEV_LCD_GPIO_SET(DEV_LCD_GPIO_DET_ZERO,GPIO_CFG_NO_PULL,GPIO_CFG_OUTPUT);
			DEV_LCD_GPIO_LO(DEV_LCD_GPIO_DET_ZERO);			
			DEV_LCD_GPIO_SET(DEV_LCD_GPIO_DET_FIRST,GPIO_CFG_NO_PULL,GPIO_CFG_OUTPUT);
		    DEV_LCD_GPIO_LO(DEV_LCD_GPIO_DET_FIRST);
			DEV_LCD_GPIO_SET(DEV_LCD_GPIO_DET_SECOND,GPIO_CFG_NO_PULL,GPIO_CFG_OUTPUT);
		    DEV_LCD_GPIO_LO(DEV_LCD_GPIO_DET_SECOND);
			printk("[DEV LCD] NOT_SUPPORT !!! \n");
      break;
  }
  return do_next_judge_flg;
}


void lcd_panel_judge_sequece(void){
	if(do_lcd_panel_judge_exec(DEV_LCD_GPIO_DET_ZERO,0)){
		if(do_lcd_panel_judge_exec(DEV_LCD_GPIO_DET_FIRST,1)){
			do_lcd_panel_judge_exec(DEV_LCD_GPIO_DET_SECOND,2);
		}
	}
}
