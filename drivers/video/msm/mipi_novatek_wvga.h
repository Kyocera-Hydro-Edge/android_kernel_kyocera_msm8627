/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */
#ifndef MIPI_NOVATEK_WVGA_H
#define MIPI_NOVATEK_WVGA_H

#define OEM_PHYS_LCD_HEIGHT 87
#define OEM_PHYS_LCD_WIDTH  52

extern int lcd_off_charge_flg;
extern int oem_panel_power_on_starting;

int mipi_novatek_wvga_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel);


int dev_lcd_display_on(  struct msm_fb_data_type *mfd );
int dev_lcd_display_off( struct msm_fb_data_type *mfd );
int dev_lcdbl_hw_pwm_calc( int and );

void mipi_novatek_wvga_set_backlight_direct(unsigned char ucBacklight);

void  kernel_disp_raw_bitmap(int x,
                              int y,
                              int column_size,
                              int line_size,
                              const unsigned char *imageData,
                              char *out_dbuf);

int oem_lcd_off_charge_check( void );
extern void oem_kernel_init_disp_on( void );
extern void oem_disp_diag_wait( void );
extern void oem_lcd_backlight_ctrl( int value );


#endif
