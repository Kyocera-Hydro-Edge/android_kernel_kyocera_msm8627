/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */
/*
 *  apds990x.c - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2010 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2010 Avago Technologies
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#include <linux/gpio.h>
#include <mach/vreg.h>
#include <linux/regulator/consumer.h>
#include <mach/hs_io_ctl_a.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/apds990x.h>

#define APDS990x_DRV_NAME   "APDS-9900"
#define DRIVER_VERSION      "1.0.4"

#define APDS990x_ALS_THRESHOLD_HSYTERESIS   20

#define APDS_PROXIMITY_SENSOR_GPIO          (46)
#define GPIO_ON                              (1)
#define GPIO_OFF                             (0)


#define I2C_APDS_ADDR                       (0x72 >> 1)
#define I2C_BUS_NUMBER                      12

//#define TUNE_DEBUG

#define APDS_DEBUG        0

#if APDS_DEBUG
#define APDS_DEBUG_LOG( arg... )   printk("APDS:" arg )
#else
#define APDS_DEBUG_LOG( arg... )
#endif

#define APDS990x_ENABLE_REG 0x00
#define APDS990x_ATIME_REG  0x01
#define APDS990x_PTIME_REG  0x02
#define APDS990x_WTIME_REG  0x03
#define APDS990x_AILTL_REG  0x04
#define APDS990x_AILTH_REG  0x05
#define APDS990x_AIHTL_REG  0x06
#define APDS990x_AIHTH_REG  0x07
#define APDS990x_PILTL_REG  0x08
#define APDS990x_PILTH_REG  0x09
#define APDS990x_PIHTL_REG  0x0A
#define APDS990x_PIHTH_REG  0x0B
#define APDS990x_PERS_REG   0x0C
#define APDS990x_CONFIG_REG 0x0D
#define APDS990x_PPCOUNT_REG    0x0E
#define APDS990x_CONTROL_REG    0x0F
#define APDS990x_REV_REG    0x11
#define APDS990x_ID_REG     0x12
#define APDS990x_STATUS_REG 0x13
#define APDS990x_CDATAL_REG 0x14
#define APDS990x_CDATAH_REG 0x15
#define APDS990x_IRDATAL_REG    0x16
#define APDS990x_IRDATAH_REG    0x17
#define APDS990x_PDATAL_REG 0x18
#define APDS990x_PDATAH_REG 0x19

#define CMD_BYTE    0x80
#define CMD_WORD    0xA0
#define CMD_SPECIAL 0xE0

#define CMD_CLR_PS_INT  0xE5
#define CMD_CLR_ALS_INT 0xE6
#define CMD_CLR_PS_ALS_INT  0xE7

#define APDS990X_LUXVALUE_MAX       50000
#define APDS990X_LUXVALUE_TABLE_MAX 50

#define APDS990X_DEV_STATUS_INIT            0x00000000
#define APDS990X_DEV_STATUS_SUSPEND         0x00000001
#define APDS990X_DEV_STATUS_SUSPEND_INT     0x00000002

#define ALS_POLLING_CNT_RESET_NONE          0x00000000
#define ALS_POLLING_CNT_RESET_MAX_DATA      0x00000001
#define ALS_POLLING_CNT_RESET_DISABLE       0x00000002
#define ALS_POLLING_CNT_RESET_STORE_POLL    0x00000004
#define ALS_POLLING_CNT_RESET_STORE_TIME    0x00000008
#define ALS_POLLING_CNT_RESET_INIT          0x00000010
#define ALS_POLLING_CNT_RESET_RESUME        0x00000020

#define APDS990X_ALS_GAIN_1X    1
#define APDS990X_ALS_GAIN_8X    8
#define APDS990X_ALS_GAIN_16X   16
#define APDS990X_ALS_GAIN_120X  120

struct apds990x_data {
    struct i2c_client *client;
    struct mutex update_lock;
    struct delayed_work dwork;
    struct delayed_work    als_dwork;
    struct input_dev *input_dev_als;
    struct input_dev *input_dev_ps;

    struct wake_lock proximity_wake_lock;

    unsigned int enable;
    unsigned int atime;
    unsigned int ptime;
    unsigned int wtime;
    unsigned int ailt;
    unsigned int aiht;
    unsigned int pilt;
    unsigned int piht;
    unsigned int pers;
    unsigned int config;
    unsigned int ppcount;
    unsigned int control;

    unsigned int enable_ps_sensor;
    unsigned int enable_als_sensor;

    unsigned int ps_threshold;
    unsigned int ps_hysteresis_threshold;
    unsigned int ps_detection;
    unsigned int ps_data;

    unsigned int als_threshold_l;
    unsigned int als_threshold_h;
    unsigned int als_data;

    unsigned int als_gain;
    unsigned int als_poll_delay;
    unsigned int als_atime;

    uint32_t vreg_on;
    int32_t ps_irq;
    int32_t ps_irq_gpio;
    int32_t cdata;
    int32_t irdata;
    int32_t pdata;
    uint32_t luxValue_table[APDS990X_LUXVALUE_TABLE_MAX];
    uint32_t als_lux_ave;
    uint32_t als_polling_cnt;
    uint32_t als_mean_times;
    uint32_t als_polling_cnt_reset;
};

struct proximity_threshold{
    int lv;
    int ppcount;
    int pdrive;
    int hi;
    int lo;
};

static struct i2c_client *client_apds = NULL;

static uint16_t guc_nv_proximity_sensor_near[3] = {0x03F0,0x03F0,0x03F0};
static uint16_t guc_nv_proximity_sensor_far[3]  = {0x03D0,0x03D0,0x03D0};
static uint16_t guc_nv_photo_sensor_beamish[3]  = {0xAAAA,0xAAAA,0xAAAA};
static uint16_t guc_nv_photo_sensor_dark[3]     = {0x9000,0x9000,0x9000};
static uint16_t guc_nv_photo_sensor_b           = 0x085C;
static uint16_t guc_nv_photo_sensor_c           = 0x0158;
static uint16_t guc_nv_photo_sensor_d           = 0x02B0;
static uint16_t guc_nv_photo_sensor_ga          = 0x2856;
static uint32_t gun_nv_status                   = 0;
static atomic_t g_dev_status;

const static struct proximity_threshold proximity_thres_tbl[] = {
    {0, 3,0,0xffff,951},
    {1, 3,0,1022  ,701},
    {2, 3,0,749   ,601},
    {3, 3,0,649   ,501},
    {4, 3,0,549   ,0},
};
static struct proximity_threshold *proximity_thres;
static int proximity_max_level;

static void apds990x_reg_init_config(struct i2c_client *client);

static int apds990x_set_enable(struct i2c_client *client, int enable)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    int ret;

    APDS_DEBUG_LOG("[IN]%s\n",__func__);
    mutex_lock(&data->update_lock);

    ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_ENABLE_REG, enable);

    mutex_unlock(&data->update_lock);

    data->enable = enable;

    APDS_DEBUG_LOG("[OUT]%s ret=%d\n",__func__,ret);
    return ret;
}

static int apds990x_set_atime(struct i2c_client *client, int atime)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    int ret;
    
    mutex_lock(&data->update_lock);
    ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_ATIME_REG, atime);
    mutex_unlock(&data->update_lock);

    data->atime = atime;

    return ret;
}

static int apds990x_set_ptime(struct i2c_client *client, int ptime)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    int ret;
    
    mutex_lock(&data->update_lock);
    ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_PTIME_REG, ptime);
    mutex_unlock(&data->update_lock);

    data->ptime = ptime;

    return ret;
}

static int apds990x_set_wtime(struct i2c_client *client, int wtime)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    int ret;
    
    mutex_lock(&data->update_lock);
    ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_WTIME_REG, wtime);
    mutex_unlock(&data->update_lock);

    data->wtime = wtime;

    return ret;
}

static int apds990x_set_ailt(struct i2c_client *client, int threshold)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    int ret;
    
    mutex_lock(&data->update_lock);
    ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS990x_AILTL_REG, threshold);
    mutex_unlock(&data->update_lock);
    
    data->ailt = threshold;

    return ret;
}

static int apds990x_set_aiht(struct i2c_client *client, int threshold)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    int ret;
    
    mutex_lock(&data->update_lock);
    ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS990x_AIHTL_REG, threshold);
    mutex_unlock(&data->update_lock);
    
    data->aiht = threshold;

    return ret;
}

static int apds990x_set_pilt(struct i2c_client *client, int threshold)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    int ret;
    
    mutex_lock(&data->update_lock);
    ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS990x_PILTL_REG, threshold);
    mutex_unlock(&data->update_lock);
    
    data->pilt = threshold;

    return ret;
}

static int apds990x_set_piht(struct i2c_client *client, int threshold)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    int ret;
    
    mutex_lock(&data->update_lock);
    ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS990x_PIHTL_REG, threshold);
    mutex_unlock(&data->update_lock);
    
    data->piht = threshold;

    return ret;
}

static int apds990x_set_pers(struct i2c_client *client, int pers)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    int ret;
    
    mutex_lock(&data->update_lock);
    ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_PERS_REG, pers);
    mutex_unlock(&data->update_lock);

    data->pers = pers;

    return ret;
}

static int apds990x_set_config(struct i2c_client *client, int config)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    int ret;
    
    mutex_lock(&data->update_lock);
    ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_CONFIG_REG, config);
    mutex_unlock(&data->update_lock);

    data->config = config;

    return ret;
}

static int apds990x_set_ppcount(struct i2c_client *client, int ppcount)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    int ret;
    
    mutex_lock(&data->update_lock);
    ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_PPCOUNT_REG, ppcount);
    mutex_unlock(&data->update_lock);

    data->ppcount = ppcount;

    return ret;
}

static int apds990x_set_control(struct i2c_client *client, int control)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    int ret;
    
    mutex_lock(&data->update_lock);
    ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_CONTROL_REG, control);
    mutex_unlock(&data->update_lock);

    data->control = control;

    if ((data->control&0x03) == 0x00)
        data->als_gain = APDS990X_ALS_GAIN_1X;
    else if ((data->control&0x03) == 0x01)
        data->als_gain = APDS990X_ALS_GAIN_8X;
    else if ((data->control&0x03) == 0x02)
        data->als_gain = APDS990X_ALS_GAIN_16X;
    else
        data->als_gain = APDS990X_ALS_GAIN_120X;

    return ret;
}

static void apds990x_proximity_change_data(struct apds990x_data *data)
{
    int i;

    APDS_DEBUG_LOG("START:%s pdata:%d\n", __func__, data->pdata);
    
    if(data->pdata > proximity_thres[0].hi 
    || data->pdata < proximity_thres[proximity_max_level].lo)
    {
      APDS_DEBUG_LOG("[prox] %s: invalid param (%d)\n",__func__,data->pdata);
      return;
    }
    
    for(i=data->ps_data;i > 0;i--)
    {
      if(data->pdata > proximity_thres[i].hi){
        data->ps_data = i - 1;
        if(proximity_thres[i].ppcount != proximity_thres[data->ps_data].ppcount
        || proximity_thres[i].pdrive != proximity_thres[data->ps_data].pdrive  ){
          break;
        }
      }
      else {
        break;
      }
    }
    
    for(i=data->ps_data;i < proximity_max_level;i++)
    {
      if(data->pdata < proximity_thres[i].lo){
        data->ps_data = i + 1;
        if(proximity_thres[i].ppcount != proximity_thres[data->ps_data].ppcount
        || proximity_thres[i].pdrive != proximity_thres[data->ps_data].pdrive  ){
          break;
        }
      }
      else {
        break;
      }
    }
    
    APDS_DEBUG_LOG("END:%s data->ps_data:%d\n", __func__, data->ps_data);
}

static void apds990x_proximity_change_setting(struct apds990x_data *data, struct i2c_client *client)
{
#ifdef TUNE_DEBUG
    apds990x_set_ppcount(client, data->ppcount);
    apds990x_set_control(client, data->control);
#else
    /* set interrupt threshold */
    data->ps_threshold = proximity_thres[data->ps_data].lo;
    data->ps_hysteresis_threshold = proximity_thres[data->ps_data].hi;

    apds990x_set_pilt(client, data->ps_threshold);
    apds990x_set_piht(client, data->ps_hysteresis_threshold);

    if(data->ppcount != proximity_thres[data->ps_data].ppcount) {
        apds990x_set_ppcount(client, proximity_thres[data->ps_data].ppcount);
    }

    if((data->control >> 6) != proximity_thres[data->ps_data].pdrive) {
        apds990x_set_control(client, 0x28 | ((proximity_thres[data->ps_data].pdrive << 6) & 0xC0));
    }
#endif
    APDS_DEBUG_LOG(":%s lv:%d count:%d ctrl:%d hi:%d lo:%d\n",__func__,
                                                      data->ps_data,
                                                      data->ppcount,
                                                      data->control,
                                                      data->ps_threshold,
                                                      data->ps_hysteresis_threshold);
}

static int LuxCalculation(struct i2c_client *client, int cdata, int irdata)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    int luxValue=0;

    int IAC1=0;
    int IAC2=0;
    int IAC=0;
    int LPC=0;
    int ALIST=0;
    int GA=guc_nv_photo_sensor_ga;
    int COE_B=guc_nv_photo_sensor_b;
    int COE_C=guc_nv_photo_sensor_c;
    int COE_D=guc_nv_photo_sensor_d;
    int DF=52;
    int ATIME=data->atime;
    int ALS_GAIN=data->als_gain;

    int ATIME_MAX_CNT=((256-ATIME)*1024);

    ATIME_MAX_CNT = ATIME_MAX_CNT >= 0xFFFF ? 0xFFFF:ATIME_MAX_CNT;
    if(ATIME_MAX_CNT <= cdata)
    {
        return ATIME_MAX_CNT;
    }

    IAC1 = ((cdata*1000 - (COE_B*irdata))/1000);
    IAC2 = (((COE_C*cdata) - (COE_D*irdata))/1000);

    if (IAC1 > IAC2)
        IAC = IAC1;
    else if (IAC1 <= IAC2)
        IAC = IAC2;
    else
        IAC = 0;

    ALIST = 2720*(256-ATIME);
    LPC = (GA*DF)/(ALIST*ALS_GAIN);

    luxValue = IAC*LPC;

    return luxValue;
}

static void apds990x_reschedule_work(struct apds990x_data *data,
                      unsigned long delay)
{
    unsigned long flags;

    spin_lock_irqsave(&data->update_lock.wait_lock, flags);

    __cancel_delayed_work(&data->dwork);
    schedule_delayed_work(&data->dwork, delay);

    spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);
}

static void apds990x_put_luxValue( struct apds990x_data *data, uint32_t luxValue )
{
    APDS_DEBUG_LOG("[IN]%s\n",__func__);
    if( data == NULL )
    {
        APDS_DEBUG_LOG("[OUT]%s\n",__func__);
        return;
    }
    
    APDS_DEBUG_LOG("%s: als_polling_cnt_reset\n", __func__);
    data->als_lux_ave = luxValue;
        
    input_report_abs(data->input_dev_als, ABS_MISC, data->als_lux_ave);
    input_sync(data->input_dev_als);

    APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}

static void apds990x_als_polling_work_handler(struct work_struct *work)
{
    struct apds990x_data *data = container_of(work, struct apds990x_data, als_dwork.work);
    struct i2c_client *client=data->client;
    int luxValue=0;
    
    data->cdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_CDATAL_REG);
    data->irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_IRDATAL_REG);
    
    luxValue = LuxCalculation(client, data->cdata, data->irdata);
    
    luxValue = luxValue>0 ? luxValue : 0;
    luxValue = luxValue<APDS990X_LUXVALUE_MAX ? luxValue : APDS990X_LUXVALUE_MAX;
    
    apds990x_put_luxValue( data, (uint32_t)luxValue );
    
    APDS_DEBUG_LOG("%s: lux = %d lux_mean = %d cdata = %x  irdata = %x pdata = %x \n", __func__, luxValue, data->als_lux_ave, data->cdata, data->irdata, data->pdata);
    
    schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
}

static void apds990x_work_handler(struct work_struct *work)
{
    struct apds990x_data *data = container_of(work, struct apds990x_data, dwork.work);
    struct i2c_client *client=data->client;
    data->pdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS990x_PDATAL_REG);

    if(0 == data->pdata)
    {
        apds990x_reg_init_config(client);
    }
    else
    {
        apds990x_proximity_change_data(data);

        apds990x_proximity_change_setting(data, client);

#ifdef TUNE_DEBUG
        printk("@@@tune_proximity pulse:%d drvstr:%d raw data :%d\n",data->ppcount, (data->control >> 6),data->pdata);
        input_report_abs(data->input_dev_ps, ABS_DISTANCE, data->pdata);
#else
        input_report_abs(data->input_dev_ps, ABS_DISTANCE, data->ps_data);
#endif /* TUNE_DEBUG */
        input_sync(data->input_dev_ps);
    }

    if (data->enable_als_sensor<=0)
        apds990x_set_enable(client, (0x09 | 0x24));
    else
        apds990x_set_enable(client, (0x09 | 0x24 | 0x02));

    i2c_smbus_write_byte(client, CMD_CLR_PS_INT);

    APDS_DEBUG_LOG("%s: pdata:%d\n",__func__,data->ps_data);
    
#ifdef TUNE_DEBUG
    if(data->enable_ps_sensor > 0)
        apds990x_reschedule_work(data, msecs_to_jiffies(200));
#else
    enable_irq(MSM_GPIO_TO_INT(data->ps_irq_gpio));
#endif /* TUNE_DEBUG */
}

static irqreturn_t apds990x_interrupt(int vec, void *info)
{
    struct i2c_client *client=(struct i2c_client *)info;
    struct apds990x_data *data = i2c_get_clientdata(client);
    uint32_t dev_status = 0;

    APDS_DEBUG_LOG("==> apds990x_interrupt\n");

    disable_irq_nosync(MSM_GPIO_TO_INT(data->ps_irq_gpio));
    dev_status = atomic_read(&g_dev_status);
    if( dev_status & APDS990X_DEV_STATUS_SUSPEND )
    {
        atomic_set(&g_dev_status, dev_status|APDS990X_DEV_STATUS_SUSPEND_INT);
    } else 
    {
        apds990x_reschedule_work(data, 0);
    }
    wake_lock_timeout(&data->proximity_wake_lock, 2 * HZ);

    return IRQ_HANDLED;
}

static int32_t apds990x_ps_irq_cnt = 0;
static void apds990x_enable_ps_irq( struct apds990x_data *data )
{
    APDS_DEBUG_LOG("[IN]%s apds990x_ps_irq_cnt=%d\n",__func__,apds990x_ps_irq_cnt);
    if( apds990x_ps_irq_cnt <= 0 )
    {
        APDS_DEBUG_LOG("enable_irq\n");
        enable_irq(MSM_GPIO_TO_INT(data->ps_irq_gpio));
        enable_irq_wake(MSM_GPIO_TO_INT(data->ps_irq_gpio));
        apds990x_ps_irq_cnt++;
    }
    APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}
static void apds990x_disable_ps_irq( struct apds990x_data *data )
{
    APDS_DEBUG_LOG("[IN]%s apds990x_ps_irq_cnt=%d\n",__func__,apds990x_ps_irq_cnt);
    if( apds990x_ps_irq_cnt > 0 )
    {
        apds990x_ps_irq_cnt--;
        disable_irq(MSM_GPIO_TO_INT(data->ps_irq_gpio));
        disable_irq_wake(MSM_GPIO_TO_INT(data->ps_irq_gpio));
        APDS_DEBUG_LOG("disable_irq\n");
    }
    APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}


static ssize_t apds990x_show_enable_ps_sensor(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds990x_data *data = i2c_get_clientdata(client);
    
    return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t apds990x_store_enable_ps_sensor(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds990x_data *data = i2c_get_clientdata(client);
    unsigned long val = simple_strtoul(buf, NULL, 10);
    unsigned long flags;
    static atomic_t serial = ATOMIC_INIT(0);

    APDS_DEBUG_LOG("%s: enable ps senosr ( %ld)\n", __func__, val);
    
    if ((val != 0) && (val != 1)) {
        APDS_DEBUG_LOG("%s:store unvalid value=%ld\n", __func__, val);
        return count;
    }
    
    if(val == 1) {
        if (data->enable_ps_sensor<=0) {

            data->enable_ps_sensor=1;
            data->ps_data = proximity_max_level;
            data->ps_threshold = 0;
            data->ps_hysteresis_threshold = 0;
            data->ps_detection = 0;

            apds990x_reg_init_config(client);

            input_report_abs(data->input_dev_ps, ABS_MISC, atomic_inc_return(&serial));

            if (data->enable_als_sensor<=0)
                apds990x_set_enable(client, (0x09 | 0x24));
            else
                apds990x_set_enable(client, (0x09 | 0x24 | 0x02));

            apds990x_enable_ps_irq(data);
#ifdef TUNE_DEBUG
            apds990x_reschedule_work(data, msecs_to_jiffies(200));
#endif /* TUNE_DEBUG */
        } else
        {
            data->enable_ps_sensor++;
        }
    } 
    else {
        data->enable_ps_sensor--;
        
        if (data->enable_ps_sensor>0) {
            ;
        } else if(data->enable_als_sensor) {
            
            apds990x_disable_ps_irq(data);
            apds990x_set_enable(client, 0x0A);
            i2c_smbus_write_byte(client, CMD_SPECIAL|0x07);
            apds990x_set_enable(client, 0xB);
            
            if( data->ps_detection != 0 )
            {
                data->ps_detection = 0;
                input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);
                input_sync(data->input_dev_ps);
            }

            spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
            
            __cancel_delayed_work(&data->als_dwork);
            schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
            
            spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);    
            
        }
        else {
            apds990x_disable_ps_irq(data);
            i2c_smbus_write_byte_data(client, CMD_BYTE|APDS990x_ENABLE_REG, 0x08);
            i2c_smbus_write_byte(client, CMD_SPECIAL|0x07);
            apds990x_set_enable(client, 0);

            if( data->ps_detection != 0 )
            {
                data->ps_detection = 0;
                input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);
                input_sync(data->input_dev_ps);
            }
        }
    }
    
    
    return count;
}

static DEVICE_ATTR(enable_ps_sensor, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
                   apds990x_show_enable_ps_sensor, apds990x_store_enable_ps_sensor);

static ssize_t apds990x_show_enable_als_sensor(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds990x_data *data = i2c_get_clientdata(client);
    
    return sprintf(buf, "%d\n", data->enable_als_sensor);
}

static ssize_t apds990x_store_enable_als_sensor(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds990x_data *data = i2c_get_clientdata(client);
    unsigned long val = simple_strtoul(buf, NULL, 10);
    unsigned long flags;
    static atomic_t serial = ATOMIC_INIT(0);
    
    APDS_DEBUG_LOG("%s: enable als sensor ( %ld)\n", __func__, val);
    
    if ((val != 0) && (val != 1))
    {
        APDS_DEBUG_LOG("%s: enable als sensor=%ld\n", __func__, val);
        return count;
    }
    
    if(val == 1) {
        if (data->enable_als_sensor<=0) {

            data->enable_als_sensor = 1;
        
            apds990x_reg_init_config(client);

            input_report_abs(data->input_dev_als, ABS_VOLUME, atomic_inc_return(&serial));

            if (data->enable_ps_sensor<=0)
                apds990x_set_enable(client, (0x09 | 0x02));
            else
                apds990x_set_enable(client, (0x09 | 0x02 | 0x24));
        
            spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
        
            __cancel_delayed_work(&data->als_dwork);
            schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
        
            spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);
        }
        else
        {
            data->enable_als_sensor++;
        }
    }
    else {
        data->enable_als_sensor--;
        if( data->enable_als_sensor > 0)
        {
            ;
        }
        else {
            if (data->enable_ps_sensor<=0) {
                apds990x_set_enable(client, 0);
            }
        
            spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
            
            __cancel_delayed_work(&data->als_dwork);
            
            spin_unlock_irqrestore(&data->update_lock.wait_lock, flags); 
            
            data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_DISABLE;
        }
    }
    
    return count;
}

static DEVICE_ATTR(enable_als_sensor, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
                   apds990x_show_enable_als_sensor, apds990x_store_enable_als_sensor);

static ssize_t apds990x_show_als_poll_delay(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds990x_data *data = i2c_get_clientdata(client);
    
    return sprintf(buf, "%d\n", data->als_poll_delay*1000);
}

static ssize_t apds990x_store_als_poll_delay(struct device *dev,
                    struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds990x_data *data = i2c_get_clientdata(client);
    unsigned long val = simple_strtoul(buf, NULL, 10);
    unsigned long flags;
    
    if( val < (1000 / APDS990X_LUXVALUE_TABLE_MAX) * 1000 )
        val = (1000 / APDS990X_LUXVALUE_TABLE_MAX) * 1000;
    
    data->als_poll_delay = val/1000;
    
    data->als_mean_times = 1000 / data->als_poll_delay;
    data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_STORE_POLL;

    spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
        
    __cancel_delayed_work(&data->als_dwork);
    schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
            
    spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);    
    
    return count;
}

static DEVICE_ATTR(als_poll_delay, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
                   apds990x_show_als_poll_delay, apds990x_store_als_poll_delay);

static struct attribute *apds990x_attributes[] = {
    &dev_attr_enable_ps_sensor.attr,
    &dev_attr_enable_als_sensor.attr,
    &dev_attr_als_poll_delay.attr,
    NULL
};


static const struct attribute_group apds990x_attr_group = {
    .attrs = apds990x_attributes,
};

#ifdef TUNE_DEBUG
static ssize_t
proximity_drvstr_show(struct device *dev,struct device_attribute *attr,char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds990x_data *data = i2c_get_clientdata(client);

    printk("%s: drive strength:%d\n",__func__,(data->control >> 6));
    return sprintf(buf, "%d\n", (data->control >> 6));
}
static ssize_t
proximity_drvstr_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds990x_data *data = i2c_get_clientdata(client);
    unsigned long val = simple_strtoul(buf, NULL, 10);

    if(count > 2) return count;

    if((0x00 <= val && val <= 0x03))
    {
	data->control &= ~(0x03 << 6);
	data->control |= (val << 6);
        printk("%s: proximity led drive strength:%d\n",__func__,(data->control >> 6));
    }
    else
    {
        printk("%s: invalid  argument %ld\n",__func__,val);
    }
    return count;
}
static ssize_t
proximity_ppulse_show(struct device *dev,struct device_attribute *attr,char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds990x_data *data = i2c_get_clientdata(client);

    printk("%s: pulse count:%d\n",__func__,data->ppcount);
    return sprintf(buf, "%d\n", data->ppcount);
}
static ssize_t
proximity_ppulse_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct apds990x_data *data = i2c_get_clientdata(client);
    unsigned long val = simple_strtoul(buf, NULL, 10);

    if(count > 4) return count;

    if((0x00 <= val && val <= 0xFF))
    {
	data->ppcount = val;
        printk("%s: proximity pulse count:%d\n",__func__,data->ppcount);
    }
    else
    {
        printk("%s: invalid  argument %ld\n",__func__,val);
    }
    return count;
}
static DEVICE_ATTR(drvstr, S_IRUGO|S_IWUGO, proximity_drvstr_show, proximity_drvstr_store);
static DEVICE_ATTR(ppulse, S_IRUGO|S_IWUGO, proximity_ppulse_show, proximity_ppulse_store);

static struct attribute *proximity_tune_debug_attributes[] = {
    &dev_attr_drvstr.attr,
    &dev_attr_ppulse.attr,
    NULL
};

static struct attribute_group proximity_tune_debug_attribute_group = {
    .attrs = proximity_tune_debug_attributes
};
#endif /* TUNE_DEBUG */


static void apds990x_store_als_mean_times( struct apds990x_data *data, uint32_t mean_times )
{
    unsigned long flags;

    APDS_DEBUG_LOG("[IN]%s\n",__func__);
    if( mean_times == 0 )
    {
        APDS_DEBUG_LOG("%s: bad param\n", __func__);
        return;
    }
    else if( mean_times > APDS990X_LUXVALUE_TABLE_MAX )
    {
        data->als_mean_times = APDS990X_LUXVALUE_TABLE_MAX;
    }
    else
    {
        data->als_mean_times = mean_times;
    }
    data->als_poll_delay = 1000 / data->als_mean_times;
    data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_STORE_TIME;
    
    spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
    
    __cancel_delayed_work(&data->als_dwork);
    schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
    
    spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);    
    APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}

static void apds990x_set_sensor_nv( unsigned long ulArg )
{
    T_APDS990X_IOCTL_NV* nv_data_type 
                            = (T_APDS990X_IOCTL_NV*)ulArg;
    APDS_DEBUG_LOG("[IN]%s\n",__func__);
    switch(nv_data_type->ulItem)
    {
    case en_NV_PROXIMITY_SENSOR_NEAR_I:
        gun_nv_status |= (0x01<<en_NV_PROXIMITY_SENSOR_NEAR_I);
        memcpy(guc_nv_proximity_sensor_near,
                nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        break;
    case en_NV_PROXIMITY_SENSOR_FAR_I:
        gun_nv_status |= (0x01<<en_NV_PROXIMITY_SENSOR_FAR_I);
        memcpy(guc_nv_proximity_sensor_far,
                nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        break;
    case en_NV_PHOTO_SENSOR_BEAMISH_I:
        gun_nv_status |= (0x01<<en_NV_PHOTO_SENSOR_BEAMISH_I);
        memcpy(guc_nv_photo_sensor_beamish,
                nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        break;
    case en_NV_PHOTO_SENSOR_DARK_I:
        gun_nv_status |= (0x01<<en_NV_PHOTO_SENSOR_DARK_I);
        memcpy(guc_nv_photo_sensor_dark,
                nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        break;
    case en_NV_PHOTO_SENSOR_B_I:
        gun_nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B_I);
        memcpy(&guc_nv_photo_sensor_b,
                nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        break;
    case en_NV_PHOTO_SENSOR_C_I:
        gun_nv_status |= (0x01<<en_NV_PHOTO_SENSOR_C_I);
        memcpy(&guc_nv_photo_sensor_c,
                nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        break;
    case en_NV_PHOTO_SENSOR_D_I:
        gun_nv_status |= (0x01<<en_NV_PHOTO_SENSOR_D_I);
        memcpy(&guc_nv_photo_sensor_d,
                nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        break;
    case en_NV_PHOTO_SENSOR_GA_I:
        gun_nv_status |= (0x01<<en_NV_PHOTO_SENSOR_GA_I);
        memcpy(&guc_nv_photo_sensor_ga,
                nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        break;
    default :
        APDS_DEBUG_LOG(KERN_ERR "set_sensor_nv: Can't set nv data\n");
        break;
    }
    APDS_DEBUG_LOG("[OUT]%s\n",__func__);
}

static int apds990x_open(struct inode *inode_type, struct file *file)
{
    APDS_DEBUG_LOG("[IN]apds990x_open\n");
    APDS_DEBUG_LOG("[OUT]apds990x_open\n");
    return 0;
}

static int apds990x_release(struct inode *inode_type, struct file *file)
{
    APDS_DEBUG_LOG("[IN]apds990x_release\n");
    APDS_DEBUG_LOG("[OUT]apds990x_release\n");
    return 0;
}

static long apds990x_ioctl(struct file *file_type, 
                          unsigned int unCmd, unsigned long ulArg)
{
    int32_t nRet = -EINVAL;
    T_APDS990X_IOCTL_PS_DETECTION ps_detection_type;
    T_APDS990X_IOCTL_ALS_MEAN_TIMES als_mean_times_type;
    T_APDS990X_IOCTL_ALS_LUX_AVE als_lux_ave_type;
    struct apds990x_data *data = i2c_get_clientdata(client_apds);

    APDS_DEBUG_LOG("[IN]apds990x_ioctl\n");
    
    memset((void*)&ps_detection_type, 0,
                        sizeof(T_APDS990X_IOCTL_PS_DETECTION) );
    memset((void*)&als_mean_times_type, 0,
                        sizeof(T_APDS990X_IOCTL_ALS_MEAN_TIMES) );
    memset((void*)&als_lux_ave_type, 0,
                        sizeof(T_APDS990X_IOCTL_ALS_LUX_AVE) );
    switch( unCmd )
    {
        case IOCTL_PS_DETECTION_GET:
            APDS_DEBUG_LOG("IOCTL_PS_DETECTION_GET START\n" );
            nRet = copy_from_user(&ps_detection_type, 
                    (void __user *)ulArg, sizeof(T_APDS990X_IOCTL_PS_DETECTION) );
            if (nRet) {
                APDS_DEBUG_LOG("error : apds990x_ioctl(unCmd = IOCTL_PS_DETECTION_GET)\n" );
                return -EFAULT;
            }
            ps_detection_type.ulps_detection = data->ps_detection;
            nRet = copy_to_user((void *)(ulArg),
                     &ps_detection_type, sizeof(T_APDS990X_IOCTL_PS_DETECTION) );
            if (nRet) {
                APDS_DEBUG_LOG("error : apds990x_ioctl(unCmd = IOCTL_PS_DETECTION_GET)\n" );
                return -EFAULT;
            }
            APDS_DEBUG_LOG("IOCTL_PS_DETECTION_GET END\n" );
            break;
        case IOCTL_ALS_MEAN_TIMES_SET:
            nRet = copy_from_user(&als_mean_times_type, 
                    (void __user *)ulArg, sizeof(T_APDS990X_IOCTL_ALS_MEAN_TIMES) );
            if (nRet) {
                APDS_DEBUG_LOG("error : apds990x_ioctl(unCmd = IOCTL_ALS_MEAN_TIMES_SET)\n" );
                return -EFAULT;
            }
            apds990x_store_als_mean_times( data, als_mean_times_type.ulals_mean_times );
            nRet = copy_to_user((void *)(ulArg),
                     &als_mean_times_type, sizeof(T_APDS990X_IOCTL_ALS_MEAN_TIMES) );
            if (nRet) {
                APDS_DEBUG_LOG("error : apds990x_ioctl(unCmd = IOCTL_ALS_MEAN_TIMES_SET)\n" );
                return -EFAULT;
            }
            break;
        case IOCTL_ALS_LUX_AVE_GET:
            nRet = copy_from_user(&als_lux_ave_type, 
                    (void __user *)ulArg, sizeof(T_APDS990X_IOCTL_ALS_LUX_AVE) );
            if (nRet) {
                APDS_DEBUG_LOG("error : apds990x_ioctl(unCmd = IOCTL_ALS_LUX_AVE_GET)\n" );
                return -EFAULT;
            }
            als_lux_ave_type.ulals_lux_ave = data->als_lux_ave;
            als_lux_ave_type.lcdata = data->cdata;
            als_lux_ave_type.lirdata = data->irdata;
            nRet = copy_to_user((void *)(ulArg),
                     &als_lux_ave_type, sizeof(T_APDS990X_IOCTL_ALS_LUX_AVE) );
            if (nRet) {
                APDS_DEBUG_LOG("error : apds990x_ioctl(unCmd = IOCTL_ALS_LUX_AVE_GET)\n" );
                return -EFAULT;
            }
            break;
        case IOCTL_APDS990X_NV_DATA_SET:
            {
                T_APDS990X_IOCTL_NV sensor_nv_type;
                memset((void*)&sensor_nv_type, 0,
                            sizeof(T_APDS990X_IOCTL_NV) );
                nRet = copy_from_user(&sensor_nv_type, 
                        (void __user *)ulArg, sizeof(T_APDS990X_IOCTL_NV) );
                if(!nRet)
                {
                    apds990x_set_sensor_nv((unsigned long)&sensor_nv_type);
                    nRet = 0;
                }
            }
            break;
        default:
            break;
    }
    APDS_DEBUG_LOG("[OUT]apds990x_ioctl nRet=%d\n",nRet);
    return nRet;
}

static struct file_operations apds990x_fops = {
    .owner = THIS_MODULE,
    .open = apds990x_open,
    .release = apds990x_release,
    .unlocked_ioctl = apds990x_ioctl,
};

static struct miscdevice apds990x_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = APDS990x_DRV_NAME,
    .fops = &apds990x_fops,
};

static int apds990x_verify_device_connection(struct i2c_client *client)
{
    int id;
    int err = 0;

    id = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS990x_ID_REG);
    if (id == 0x20) {
        APDS_DEBUG_LOG("APDS-9901\n");
    }
    else if (id == 0x29) {
        APDS_DEBUG_LOG("APDS-990x\n");
    }
    else if (id == 0x30) {
        APDS_DEBUG_LOG("APDS-993x\n");
    }
    else if (id == 0x39) {
        APDS_DEBUG_LOG("APDS-9930\n");
    }
    else {
        APDS_DEBUG_LOG("None of APDS-9901, APDS-990x and APDS-993x id=0x%x\n",id);
        err = -EIO;
    }

    return err;
}

static void apds990x_reg_init_config(struct i2c_client *client)
{
    apds990x_set_atime(client, 0xdb);
    apds990x_set_ptime(client, 0xff);
    apds990x_set_wtime(client, 0xff);
    apds990x_set_ppcount(client, proximity_thres[proximity_max_level].ppcount);
    apds990x_set_config(client, 0x00);
    apds990x_set_pers(client, 0x11);

    apds990x_set_ailt(client, 0);
    apds990x_set_aiht(client, 0xffff);
    apds990x_set_pilt(client, 0);
    apds990x_set_piht(client, 0);

    apds990x_set_control(client, 0x28 | ((proximity_thres[proximity_max_level].pdrive << 6) & 0xC0));
}

static int apds990x_init_client(struct i2c_client *client)
{
    struct apds990x_data *data = i2c_get_clientdata(client);

    APDS_DEBUG_LOG("[IN]%s\n",__func__);

    data->enable = 0;
    data->ps_threshold = 0;
    data->ps_hysteresis_threshold = 0;
    data->ps_detection = 0;
    data->enable_als_sensor = 0;
    data->enable_ps_sensor = 0;
    data->als_poll_delay = 250;
    data->als_atime = 0xdb;
    
    memset( data->luxValue_table, 0, sizeof(data->luxValue_table) );
    data->als_lux_ave = 0;
    data->als_polling_cnt = 0;
    data->als_mean_times = 4;
    data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_INIT;

    data->enable_ps_sensor = 1;
    data->enable_als_sensor = 1;

    data->ps_threshold = 0;
    data->ps_hysteresis_threshold = 0;

    data->enable_ps_sensor = 0;
    data->enable_als_sensor = 0;

    proximity_thres = (struct proximity_threshold *)proximity_thres_tbl;
    proximity_max_level = ARRAY_SIZE(proximity_thres_tbl) - 1;

    apds990x_reg_init_config(client);
    apds990x_set_enable(client, 0);

    APDS_DEBUG_LOG("[OUT]%s\n",__func__);

    return 0;
}


static struct i2c_driver apds990x_driver;
static int __devinit apds990x_probe(struct i2c_client *client,
                   const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct apds990x_data *data;
    int err = 0;

    APDS_DEBUG_LOG("[IN]%s\n",__func__);
    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
        err = -EIO;
        goto exit;
    }

    if(apds990x_verify_device_connection(client)) {
        err = -ENOTCONN;
        goto exit; 
    }

    data = kzalloc(sizeof(struct apds990x_data), GFP_KERNEL);
    if (!data) {
        err = -ENOMEM;
        goto exit;
    }
    data->client = client;
    i2c_set_clientdata(client, data);

    data->ps_irq_gpio = APDS_PROXIMITY_SENSOR_GPIO;
    
    err = gpio_request(data->ps_irq_gpio,
                        APDS990x_DRV_NAME);
    APDS_DEBUG_LOG("gpio_request err=%d\n",err);
    if (err < 0) {
        APDS_DEBUG_LOG("[%s] failed to request GPIO=%d, ret=%d\n",
               __FUNCTION__,
               data->ps_irq_gpio,
               err);
        goto exit_kfree;
    }
    err = gpio_direction_input(data->ps_irq_gpio);
    APDS_DEBUG_LOG("gpio_direction_input err=%d\n",err);
    if (err < 0) {
        APDS_DEBUG_LOG("[%s] failed to configure direction for GPIO=%d, ret=%d\n",
               __FUNCTION__,
               data->ps_irq_gpio,
               err);
        goto exit_kfree;
    }

    err = request_any_context_irq(MSM_GPIO_TO_INT(data->ps_irq_gpio), apds990x_interrupt, IRQ_TYPE_LEVEL_LOW,
        APDS990x_DRV_NAME, (void *)client);
    APDS_DEBUG_LOG("request_any_context_irq err=%d\n",err);
    if(err < 0) {
        APDS_DEBUG_LOG("%s Could not allocate APDS990x_INT(%d) ! err=%d\n",
                 __func__,data->ps_irq_gpio,err);
    
        goto exit_kfree;
    }
    disable_irq(MSM_GPIO_TO_INT(data->ps_irq_gpio));

    APDS_DEBUG_LOG("%s interrupt is hooked\n", __func__);


    INIT_DELAYED_WORK(&data->dwork, apds990x_work_handler);
    INIT_DELAYED_WORK(&data->als_dwork, apds990x_als_polling_work_handler);
 
    mutex_init(&data->update_lock);

    wake_lock_init(&data->proximity_wake_lock, WAKE_LOCK_SUSPEND, "proximity");


    err = apds990x_init_client(client);
    if (err)
    {
        APDS_DEBUG_LOG("Failed apds990x_init_client\n");
        goto exit_free_irq;
    }


    data->input_dev_als = input_allocate_device();
    if (!data->input_dev_als) {
        err = -ENOMEM;
        APDS_DEBUG_LOG("Failed to allocate input device als\n");
        goto exit_free_irq;
    }

    data->input_dev_ps = input_allocate_device();
    if (!data->input_dev_ps) {
        err = -ENOMEM;
        APDS_DEBUG_LOG("Failed to allocate input device ps\n");
        goto exit_free_dev_als;
    }

    set_bit(EV_ABS, data->input_dev_als->evbit);
    set_bit(EV_ABS, data->input_dev_ps->evbit);

    input_set_abs_params(data->input_dev_als, ABS_MISC, 0, APDS990X_LUXVALUE_MAX, 0, 0);
    input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, proximity_max_level, 0, 0);
    input_set_capability(data->input_dev_ps, EV_ABS, ABS_MISC);
    input_set_capability(data->input_dev_als, EV_ABS, ABS_VOLUME);

    data->input_dev_als->name = "light sensor";
    data->input_dev_ps->name = "proximity sensor";

    err = input_register_device(data->input_dev_als);
    if (err) {
        err = -ENOMEM;
        APDS_DEBUG_LOG("Unable to register input device als: %s\n",
               data->input_dev_als->name);
        goto exit_free_dev_ps;
    }

    err = input_register_device(data->input_dev_ps);
    if (err) {
        err = -ENOMEM;
        APDS_DEBUG_LOG("Unable to register input device ps: %s\n",
               data->input_dev_ps->name);
        goto exit_unregister_dev_als;
    }

    err = sysfs_create_group(&client->dev.kobj, &apds990x_attr_group);
    if (err)
    {
        APDS_DEBUG_LOG("Failed sysfs_create_group\n");
        goto exit_unregister_dev_ps;
    }
#ifdef TUNE_DEBUG
    input_set_drvdata(data->input_dev_ps, data);
    err = sysfs_create_group(&data->input_dev_ps->dev.kobj, &proximity_tune_debug_attribute_group);
    if (err) {
        printk("%s: Failed sysfs_create_group for tune debug\n",__func__);
    }
#endif /* TUNE_DEBUG */


    device_init_wakeup(&client->dev, 1);
    atomic_set(&g_dev_status, APDS990X_DEV_STATUS_INIT);
    
    err = misc_register(&apds990x_device);
    if (err)
    {
        APDS_DEBUG_LOG(KERN_ERR
               "apds990x_probe: apds990x register failed\n");
        goto exit_sysfs_remove;
    }
    APDS_DEBUG_LOG("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);
    client_apds = client;
    APDS_DEBUG_LOG("[OUT]%s\n",__func__);
    return 0;

exit_sysfs_remove:
    sysfs_remove_group(&client->dev.kobj, &apds990x_attr_group);
exit_unregister_dev_ps:
    input_unregister_device(data->input_dev_ps);    
exit_unregister_dev_als:
    input_unregister_device(data->input_dev_als);
exit_free_dev_ps:
    input_free_device(data->input_dev_ps);
exit_free_dev_als:
    input_free_device(data->input_dev_als);
exit_free_irq:
    mutex_destroy(&data->update_lock);
    wake_lock_destroy(&data->proximity_wake_lock);
    free_irq(MSM_GPIO_TO_INT(data->ps_irq_gpio), client);
exit_kfree:
    kfree(data);
exit:
    APDS_DEBUG_LOG("[OUT]%s err=%d\n",__func__,err);
    return err;
}

static int __devexit apds990x_remove(struct i2c_client *client)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    
    input_unregister_device(data->input_dev_als);
    input_unregister_device(data->input_dev_ps);
    
    input_free_device(data->input_dev_als);
    input_free_device(data->input_dev_ps);

    free_irq(MSM_GPIO_TO_INT(data->ps_irq_gpio), client);

    sysfs_remove_group(&client->dev.kobj, &apds990x_attr_group);
    misc_deregister(&apds990x_device);

    data->enable_ps_sensor = 0;
    data->enable_als_sensor = 0;
    
    apds990x_set_enable(client, 0);

    kfree(data);

    return 0;
}

#ifdef CONFIG_PM

static int apds990x_suspend(struct i2c_client *client, pm_message_t mesg)
{
    APDS_DEBUG_LOG("[IN]%s\n", __func__);
    atomic_set(&g_dev_status, APDS990X_DEV_STATUS_SUSPEND);
    APDS_DEBUG_LOG("[OUT]%s\n", __func__);
    return 0;
}

static int apds990x_resume(struct i2c_client *client)
{
    struct apds990x_data *data = i2c_get_clientdata(client);
    APDS_DEBUG_LOG("[IN]%s\n", __func__);

    data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_RESUME;
    
    if(atomic_read(&g_dev_status) & APDS990X_DEV_STATUS_SUSPEND_INT)
    {
        apds990x_reschedule_work(data, 0);
    }
    
    atomic_set(&g_dev_status, APDS990X_DEV_STATUS_INIT);
    APDS_DEBUG_LOG("[OUT]%s\n", __func__);
    return 0;
}

#else

#define apds990x_suspend    NULL
#define apds990x_resume     NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id apds990x_id[] = {
    { APDS990x_DRV_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, apds990x_id);

static struct i2c_driver apds990x_driver = {
    .driver = {
        .name   = APDS990x_DRV_NAME,
        .owner  = THIS_MODULE,
    },
    .suspend = apds990x_suspend,
    .resume = apds990x_resume,
    .probe  = apds990x_probe,
    .remove = __devexit_p(apds990x_remove),
    .id_table = apds990x_id,
};

static int __init apds990x_init(void)
{
    int32_t rc;
    
    APDS_DEBUG_LOG("[IN]%s\n",__func__);
    rc = i2c_add_driver(&apds990x_driver);
    if (rc != 0) {
        APDS_DEBUG_LOG("can't add i2c driver\n");
        rc = -ENOTSUPP;
        return rc;
    }

    APDS_DEBUG_LOG("[OUT]%s\n",__func__);
    return rc;
}

static void __exit apds990x_exit(void)
{
    i2c_del_driver(&apds990x_driver);
    
    i2c_unregister_device(client_apds);
    client_apds = NULL;
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS990x ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds990x_init);
module_exit(apds990x_exit);
