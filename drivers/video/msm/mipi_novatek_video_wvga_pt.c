/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_novatek_wvga.h"

#include <mach/msm_smd.h>

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
	.regulator = {0x03, 0x0a, 0x04, 0x00, 0x20},
	.ctrl = {0x5f, 0x00, 0x00, 0x10},
	.strength = {0xff, 0x00, 0x06, 0x00},
	.timing = { 0x68, 0x27, 0x0E,
	0,
	0x34, 0x3A, 0x13, 0x2B, 0x19, 0x03, 0x04},
	.pll = { 0x00,
	0x58, 0x01, 0xD9,
	0x00, 0x50, 0x48, 0x63,
	0x40, 0x00, 0x00,
	0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },
};

int oem_lcd_off_charge_check( void )
{
  unsigned int *power_on_status;
  unsigned int size;
  power_on_status = smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &size);
  if (power_on_status)
  {
    if (*power_on_status & 0x20)
    {
      return(TRUE);
    }
  }
  return(FALSE);
}

static int __init mipi_command_novatek_wvga_pt_init(void)
{
  int ret;

  if (msm_fb_detect_client("mipi_video_novatek_wvga")){
    printk("%s() Panel Check NG!!!! \n", __func__);
    return 0;
  }
  printk("%s() VIDEO PANEL !!!!! \n", __func__);
  pinfo.xres               = 480;
  pinfo.yres               = 800;
  pinfo.type               = MIPI_VIDEO_PANEL;
  pinfo.pdest              = DISPLAY_1;
  pinfo.wait_cycle         = 0;
  pinfo.bpp                = 24;

  pinfo.lcdc.h_back_porch  = 101;
  pinfo.lcdc.h_front_porch =  23;
  pinfo.lcdc.h_pulse_width =   6;
  pinfo.lcdc.v_back_porch  =   9;
  pinfo.lcdc.v_front_porch =   5;
  pinfo.lcdc.v_pulse_width =   4;

  pinfo.lcdc.xres_pad      = 0;
  pinfo.lcdc.yres_pad      = 0;
  pinfo.lcdc.border_clr    = 0;
  pinfo.lcdc.underflow_clr = 0xff;
  pinfo.lcdc.hsync_skew    = 0;
  pinfo.bl_max             = 255;
  pinfo.bl_min             = 1;
  pinfo.fb_num             = 2;
  pinfo.clk_rate            = 359270000;

  pinfo.mipi.mode                = DSI_VIDEO_MODE;
  pinfo.mipi.pulse_mode_hsa_he   = TRUE;
  pinfo.mipi.hfp_power_stop      = TRUE;
  pinfo.mipi.hbp_power_stop      = TRUE;
  pinfo.mipi.hsa_power_stop      = FALSE;
  pinfo.mipi.eof_bllp_power_stop = TRUE;
  pinfo.mipi.bllp_power_stop     = TRUE;
  pinfo.mipi.traffic_mode        = DSI_NON_BURST_SYNCH_PULSE;
  pinfo.mipi.dst_format          = DSI_VIDEO_DST_FORMAT_RGB888;
  pinfo.mipi.vc                  = 0;
  pinfo.mipi.rgb_swap            = DSI_RGB_SWAP_RGB;
  pinfo.mipi.dlane_swap          = 0x01;
  pinfo.mipi.data_lane0          = TRUE;
  pinfo.mipi.data_lane1          = TRUE;
  pinfo.mipi.data_lane2          = FALSE;
  pinfo.mipi.data_lane3          = FALSE;
  pinfo.mipi.t_clk_post          = 0x04;
  pinfo.mipi.t_clk_pre           = 0x17;
  pinfo.mipi.stream              = 0;
  pinfo.mipi.mdp_trigger         = DSI_CMD_TRIGGER_SW;
  pinfo.mipi.dma_trigger         = DSI_CMD_TRIGGER_SW;
  pinfo.mipi.frame_rate          = 59;
  pinfo.mipi.dsi_phy_db          = &dsi_video_mode_phy_db;
  pinfo.mipi.tx_eot_append       = TRUE;
  pinfo.mipi.esc_byte_ratio      = 4;

  pinfo.mipi.force_clk_lane_hs   = FALSE;
  pinfo.mipi.no_max_pkt_size = FALSE;


  ret = mipi_novatek_wvga_device_register(&pinfo, MIPI_DSI_PRIM, MIPI_DSI_PANEL_WVGA);
  if (ret){
    pr_err("%s: failed to register device!\n", __func__);
  }

  oem_kernel_init_disp_on();
  return ret;
}

module_init(mipi_command_novatek_wvga_pt_init);
