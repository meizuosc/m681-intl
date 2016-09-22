/******************************************************************************
 * MODULE       : rohm_bh1745_i2c.c
 * FUNCTION     : Driver source for BH1745, Ambient Light Sensor(RGB) IC
 * AUTHOR       : Shengfan Wen
 * PROGRAMMED   : Software Development & Consulting, ArcharMind
 * MODIFICATION : Modified by Shengfan Wen, DEC/24/2015
 * NOTICE       : This software had been verified using MT6795.
 *              : When you use this code and document, Please verify all
 *              : operation in your operating system.
 * REMARKS      :
 * COPYRIGHT    : Copyright (C) 2015 - ROHM CO.,LTD.
 *              : This program is free software; you can redistribute it and/or
 *              : modify it under the terms of the GNU General Public License
 *              : as published by the Free Software Foundation; either version 2
 *              : of the License, or (at your option) any later version.
 *              :
 *              : This program is distributed in the hope that it will be useful,
 *              : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *              : GNU General Public License for more details.
 *              :
 *              : You should have received a copy of the GNU General Public License
 *              : along with this program; if not, write to the Free Software
 *              : Foundation, Inc., 51 Franklin Street, Fifth Floor,Boston, MA  02110-1301, USA.
 *****************************************************************************/
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/proc_fs.h>
#include <linux/slab.h> /* For kzalloc, kfree */
#include <linux/miscdevice.h>
#include <linux/string.h>   /* For memcpy */

#include "alsps.h"
#include "cust_alsps.h"
#include "rohm_bh1745_i2c.h"

/* structure of peculiarity to use by system */
typedef struct {
    struct i2c_client       *client;            /* structure pointer for i2c bus */
    int                     use_irq;            /* flag of whether to use interrupt or not */
    u32                     tp_module_num;      /* TP module count */
    LUX_PARAMETER           lux_parameter;      /* TP module parameter for calculating lux */
#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)
    COLOR_PARAMETER         ct_parameter;       /* TP module parameter for calculating color temperature(ct) */
#endif
    int            		  tp_module_id;       /*  Touch panel manufacture id */
    COLOR_T                 color_type;         /*  Touch panel color type */
} RGB_DATA;

/* logical functions */
static int                  rgb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int                  rgb_i2c_remove(struct i2c_client *client);
static void                 bh1745_power(struct alsps_hw *hw, unsigned int on);
static int                  bh1745_calculate_light(READ_DATA_ARG data, unsigned char gain, unsigned short time);
static int                  bh1745_local_init(void);
static int                  bh1745_local_uninit(void);
static int                  bh1745_get_parameter_from_dts(struct device_node *node, RGB_DATA *rgb);
static COLOR_T              bh1745_get_color_type(void);
static  int         		   bh1745_get_tp_module_id(void);

#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)
static int                  bh1745_open(struct inode *node, struct file *file);
static int                  bh1745_release(struct inode *node, struct file *file);
static long                 bh1745_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
#endif

/* access functions */
static int rgb_driver_init(struct i2c_client *client, INIT_ARG data);
static int rgb_driver_shutdown(struct i2c_client *client);
static int rgb_driver_reset(struct i2c_client *client);
static int rgb_driver_write_power_on_off(struct i2c_client *client, unsigned char data);
static int rgb_driver_read_data(struct i2c_client *client, READ_DATA_ARG *data);

/**************************** variable declaration ****************************/

static const char               rgb_driver_ver[] = BH1745_DRIVER_VER;
static RGB_DATA                 *obj = NULL;
static unsigned short           measurement_time[] = {160, 320, 640, 1280, 2560, 5120};
static unsigned char            adc_gain[] = {1, 2, 16};
int ctp_chip_id = -1;
int ctp_chip_colour = -1;
int rohm_trace_flag = 0;
/**************************** structure declaration ****************************/

static const struct i2c_device_id rgb_id[] = {
    { BH1745_I2C_NAME, 0 }, /* rohm bh1745 driver */
    { }
};

static struct i2c_driver bh1745_driver = {
    .driver = {                     		/* device driver model driver */
        .owner = THIS_MODULE,
        .name  = BH1745_I2C_NAME,
    },
    .probe    = rgb_i2c_probe,          	/* callback for device binding */
    .remove   = rgb_i2c_remove,         	/* callback for device unbinding */
    .shutdown = NULL,
    .suspend  = NULL,
    .resume   = NULL,
    .id_table = rgb_id,             		/* list of I2C devices supported by this driver */
};
/*******************************************************************************/

#if 0
static struct alsps_init_info bh1745_init_info = {
    .name = BH1745_I2C_NAME,        		/* Alsps driver name */
    .init = bh1745_local_init,      		/* Initialize alsps driver */
    .uninit = bh1745_local_uninit,       /* Uninitialize alsps driver */
};
#endif

#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)
static struct file_operations bh1745_ops = {
    .owner = THIS_MODULE,                   		/* Owner for this operations */
    .open = bh1745_open,                   		/* Interface for open char device */
    .release = bh1745_release,              		/* Interface for release char device */
    .unlocked_ioctl = bh1745_unlocked_ioctl,		/* Interface for control char device */
};

static struct miscdevice bh1745_misc = {
    .minor = MISC_DYNAMIC_MINOR,            		/* Minor number for char device */
    .name = ROHM_CHAR_NAME,                 	/* Char device name */
    .fops = &bh1745_ops,                    	/* Char device operations */
};
#endif

/************************************************************
 *  logic function 
 ***********************************************************/
#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)
static int bh1745_open(struct inode *node, struct file *file)
{
	BH1745_FUN();
    return 0;
}

static int bh1745_release(struct inode *node, struct file *file)
{
	BH1745_FUN();
    return 0;
}

/*********************************************************************************
 * @Brief: bh1745_unlocked_ioctl Interface for control char device
 * @Param: file Pointer for file structure.
 * @Param: cmd  Control command
 * @Param: arg  Control argument
 * @Returns: 0 for success,other for failed.
 *********************************************************************************/
static long bh1745_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int              result;
    COLOR_PARAMETER  *parameter;
    COLOR_PARAMETER  *tp_parameter;
    
	 BH1745_FUN();
    if (NULL == obj)
    {
        BH1745_ERR(" Parameter error \n");
        return EINVAL;
    }

    switch (cmd)
    {
        case ROHM_GET_PARAMETER:
            BH1745_INFO("ioctl get parameter.\n");

            parameter = (COLOR_PARAMETER *)arg;

            tp_parameter = &obj->ct_parameter;

            /* Copy parameter to userspace */
            result = copy_to_user(parameter, tp_parameter, sizeof(COLOR_PARAMETER));
            if (result)
            {
                BH1745_ERR( "Copy to userspace failed\n");
                return result;
            }

            break;

        default:
            return -EINVAL;
    }

    return 0;
}

#endif

static void bh1745_power(struct alsps_hw *hw, unsigned int on)
{
    static unsigned int power_on;
	BH1745_FUN();

    if (hw->power_id != MT65XX_POWER_NONE) {
        if (power_on == on)
            BH1745_INFO("ignore power control: %d\n", on); 
        else if (on) {
            if (!hwPowerOn(hw->power_id, hw->power_vol, BH1745_I2C_NAME))
                BH1745_ERR("power on fails!!\n");
        }
        else {
            if (!hwPowerDown(hw->power_id, BH1745_I2C_NAME))
                BH1745_ERR("power off fail!!\n");
        }
    }
    power_on = on;
}

/*********************************************************************************
 * @Brief: bh1745_open_report_data BH1745 initialization or uninitialization
 * @Param: open 1 for initialize,0 for uninitialize
 * @Returns: 0 for success,other for failed.
 *********************************************************************************/
int bh1745_open_report_data(int open)
{
	BH1745_FUN();
    return 0;
}
EXPORT_SYMBOL_GPL(bh1745_open_report_data);

/*********************************************************
 * @Brief: bh1745_enable_nodata Enable or disable BH1745
 * @Param: en 1 for enable,0 for disable
 * @Returns: 0 for success,others for failed.
 *********************************************************/
int bh1745_enable_nodata(int en)
{
	BH1745_FUN();
    if (NULL == obj)
    {
        BH1745_ERR(" Parameter error \n");
        return EINVAL;
    }
    return rgb_driver_write_power_on_off(obj->client, en);
}
EXPORT_SYMBOL_GPL(bh1745_enable_nodata);

/*******************************************************
 * @Brief: bh1745_set_delay Set delay,not used for now.
 * @Param: delay Delay time.
 *******************************************************/
int bh1745_set_delay(u64 delay)
{
	BH1745_FUN();
    return 0;
}
EXPORT_SYMBOL_GPL(bh1745_set_delay);

/*********************************************************************************
 * @Brief: bh1745_get_tp_module_id Get tp module id,use it to get right parameters.
 * @Returns: Tp module id.
 *********************************************************************************/
static int bh1745_get_tp_module_id(void)
{
	BH1745_FUN();

	if(ctp_chip_id == 84){
		BH1745_WARNING("ctp is ft5436\n");
		return 0;
	}else if(ctp_chip_id == 37212){
		BH1745_WARNING("ctp is gt9xx\n");
		return 1;
	}else{
		BH1745_WARNING("ctp is unknow\n");
		return -1;
	}
}

/************************************************************************************
 * @Brief: bh1745_get_color_type Get color type which use it to get right parameters.
 * @Returns: Color type
 ************************************************************************************/
static COLOR_T bh1745_get_color_type(void)
{
	BH1745_FUN();

	if(ctp_chip_colour == 1)
	{
		BH1745_WARNING("ctp colour is white\n");
		return WHITE;
	}
	else if(ctp_chip_colour == 0)
	{
		BH1745_WARNING("ctp colour is black\n");
		return BLACK;
	}
	else
	{
		BH1745_WARNING("ctp colour is unknow\n");
		return INVALID_COLOUR;
	}
}

/*************************************************************
 * @Brief: bh1745_calculate_light Calculate lux base on rgbc
 * @Param: data RGBC value from sensor
 * @Returns: lux value or failed.
 *************************************************************/
static int bh1745_calculate_light(READ_DATA_ARG data,    unsigned char     gain, unsigned short time)
{
    unsigned long        lx;
    unsigned long        lx_tmp;
    LUX_PARAMETER        *parameter = NULL;

    if (NULL == obj)
    {
        BH1745_ERR(" Parameter error \n");
        return EINVAL;
    }

    parameter = &obj->lux_parameter;

	if(rohm_trace_flag == 1)
	{
   		BH1745_WARNING("Judge:%u,red[0]:%u,red[1]:%u,green[0]:%u,green[1]:%u\n",
        parameter->judge,parameter->red[0], parameter->red[1],
        parameter->green[0], parameter->green[1]);
	}

    /* Lux calculation */
    if (data.green < 1)
    {
        lx = 0;
    }
    else
    {
        if ((data.clear*JUDGE_FIXED_COEF) < (parameter->judge*data.green))
		 {
            lx_tmp = parameter->red[0] * data.red + parameter->green[0] * data.green;
		 }
        else
		 {
            lx_tmp = parameter->red[1] * data.red + parameter->green[1] * data.green;
		 }

        lx = ((lx_tmp*160 *TRANS / gain) / time) /CUT_UNIT;
    }

	if(rohm_trace_flag == 1)
	{
    	BH1745_WARNING("Red:%d,Green:%d,Blue:%d,Clear:%d,Light calculation:%lu\n",
            data.red, data.green, data.blue, data.clear, lx);
	}

    return (lx);
}

/*********************************************************
 * @Brief: bh1745_get_data Get data from BH1745 hardware.
 * @Param: als_value Return value including lux and rgbc.
 * @Param: status Return bh1745 status.
 * @Returns: 0 for success,other for failed.
 *********************************************************/
int bh1745_get_data(int *als_value, int *status)
{
	static int pre_alsvalue;
    int result;
    unsigned char       gain;
    unsigned short      time;
    READ_DATA_ARG       data;

    if(als_value == NULL || status == NULL || obj == NULL)
    {
        BH1745_ERR(" Parameter error \n");
        return -EINVAL;
    }

/* set default value to ALSPS_INVALID_VALUE if it can't get an valid data */
#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)
    als_value[0] = pre_alsvalue; 
#else
    *als_value = pre_alsvalue;
#endif
    result = i2c_smbus_read_byte_data(obj->client, REG_MODECONTROL2);
    if (result < 0)
    {
        BH1745_ERR("Read data from IC error.\n");
        return result;
    }

	if(rohm_trace_flag == 1)
	{
    	BH1745_WARNING("bh1745_get_data REG_MODECONTROL2 = 0x%x\n", result);
	}

    if ((result & RGBC_VALID_HIGH) == 0)
    {
    	if(rohm_trace_flag == 1)
		{
        	BH1745_ERR("Data error.\n");
    	}
        return -1;
    }

    gain = adc_gain[result & 0x3];

    result = i2c_smbus_read_byte_data(obj->client, REG_MODECONTROL1);
    if (result < 0)
    {
        BH1745_ERR("Read data from IC error.\n");
        return result;
    }

	if(rohm_trace_flag == 1)
	{
    	BH1745_WARNING("bh1745_get_data REG_MODECONTROL1 = 0x%x\n", result);
	}

    time = measurement_time[result & 0x7];

    /* read rgbc data */
    result = rgb_driver_read_data(obj->client, &data);
    if (result)
    {
        BH1745_ERR("Read data from bh1745 failed.\n");
        return result;
    }

	if(rohm_trace_flag == 1)
	{
   		BH1745_WARNING("bh1745_get_data gain = 0x%x, time = 0x%x\n", gain,time);
	}

#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)
    als_value[0] = bh1745_calculate_light(data, gain, time);
    als_value[1] = data.red;
    als_value[2] = data.green;
    als_value[3] = data.blue;
    als_value[4] = data.clear;
#else
    *als_value = bh1745_calculate_light(data, gain, time);
	pre_alsvalue = *als_value;
#endif

    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

    return 0;
}
EXPORT_SYMBOL_GPL(bh1745_get_data);

/**************************************************
 * @Brief: bh1745_local_init Initial BH1745 driver.
 * @Returns: 0 for success,others for failed.
 **************************************************/
static int bh1745_local_init(void)
{
	BH1745_FUN();
    struct alsps_hw *hw = get_cust_alsps_hw();

    bh1745_power(hw, 1);
    return i2c_add_driver(&bh1745_driver);
}

/****************************************************
 * @Brief: bh1745_local_uninit Remove BH1745 driver.
 ****************************************************/
static int bh1745_local_uninit(void)
{
	BH1745_FUN();
    struct alsps_hw *hw = get_cust_alsps_hw();

    bh1745_power(hw, 0);
    i2c_del_driver(&bh1745_driver);

    return 0;
}

static int bh1745_get_parameter_from_dts(struct device_node *node, RGB_DATA *rgb)
{
    int             result;
    int             length;
    char            node_name[1024];
    CALC_LUX_PARAMETER      *calc_lux_parameter = NULL;
    LUX_PARAMETER           *lux_coefficient_tmp = NULL;

    struct property *property = NULL;

#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)
    const char      *data;
    int             index;
    CALC_COLOR_PARAMETER    *calc_color_parameter = NULL;
    COLOR_PARAMETER         *color_coefficient_tmp = NULL;
#endif

    if (NULL == node || NULL == rgb)
    {
        BH1745_ERR( "No device tree entry.\n");
        return -EINVAL;
    }

    /* Read tp vendor count from dts file. */
    result = of_property_read_u32(node, TP_MODULE_COUNT, &rgb->tp_module_num);
	BH1745_ERR( "rohm1745 tp_module_id:%d.\n", rgb->tp_module_id);

    if (result)
    {
        BH1745_ERR( "Read tp module number from dts failed, use default number.\n");
        rgb->tp_module_num = TP_MODULE_COUNT_DEFAULT;
    }

    if(rgb->tp_module_num < rgb->tp_module_id)
    {
        BH1745_ERR( "Invalid tp_module_id:%d.\n", rgb->tp_module_id);
        return -EINVAL;
    }

    /* Memory allocation. */
    calc_lux_parameter = kzalloc(sizeof(CALC_LUX_PARAMETER), GFP_KERNEL);
    if (!calc_lux_parameter)
    {
        BH1745_ERR( "Memory allocation failed.\n");
        return -ENOMEM;
    }

    /* Read lux parameter from dts file. */
    memset(node_name, 0, sizeof(node_name));
    sprintf(node_name, TP_LUX_PARAMETER_PREFIX "%d", rgb->tp_module_id);
    BH1745_INFO("node name = %s\n", node_name);

    /* Try to find node,if it exists,get its value length */
    property = of_find_property(node, node_name, &length);
    if (property)
    {
        /* If length does not match structure's */
        if (length != sizeof(CALC_LUX_PARAMETER))
        {
            BH1745_ERR( "Parameter length did not match.Please fix!!!\n");
            result = -EINVAL;
            goto err_read_calc_lux_parameter_failed;
        }

        /* Read parameter */
        result = of_property_read_u32_array(node, node_name, (u32 *)calc_lux_parameter, length/sizeof(u32));
        if (result)
        {
            BH1745_ERR( "Read parameter from dts failed.\n");
            goto err_read_calc_lux_parameter_failed;
        }

        BH1745_WARNING("module id : %u\n", calc_lux_parameter->module_id);
        
        BH1745_WARNING("gold judge = %u, red[0] = %u, red[1] = %u, green[0] = %u, green[1] = %u\n",
                calc_lux_parameter->gold_parameter.judge,
                calc_lux_parameter->gold_parameter.red[0],
                calc_lux_parameter->gold_parameter.red[1],
                calc_lux_parameter->gold_parameter.green[0],
                calc_lux_parameter->gold_parameter.green[1]
        );
              
        BH1745_WARNING("white judge = %u, red[0] = %u, red[1] = %u, green[0] = %u, green[1] = %u\n",
                calc_lux_parameter->white_parameter.judge,
                calc_lux_parameter->gold_parameter.red[0],
                calc_lux_parameter->gold_parameter.red[1],
                calc_lux_parameter->gold_parameter.green[0],
                calc_lux_parameter->gold_parameter.green[1]
        );

        BH1745_WARNING("black judge = %u, red[0] = %u, red[1] = %u, green[0] = %u, green[1] = %u\n",
                calc_lux_parameter->black_parameter.judge,
                calc_lux_parameter->black_parameter.red[0],
                calc_lux_parameter->black_parameter.red[1],
                calc_lux_parameter->black_parameter.green[0],
                calc_lux_parameter->black_parameter.green[1]
        );

        /* Get parameter accord to color type */
        if (GOLD == rgb->color_type)
        {
            lux_coefficient_tmp = &calc_lux_parameter->gold_parameter;
        }
        else if (WHITE == rgb->color_type)
        {
            lux_coefficient_tmp = &calc_lux_parameter->white_parameter;
        }
        else if (BLACK == rgb->color_type)
        {
            lux_coefficient_tmp = &calc_lux_parameter->black_parameter;
        }

        //copy valid coefficent to rgb struct
        memcpy(&rgb->lux_parameter, lux_coefficient_tmp, sizeof(LUX_PARAMETER) );

        //free calc_lux_parameter
        kfree(calc_lux_parameter);
    }
    else
    {
        BH1745_ERR( "Cannot find dts node : %s\n", node_name);
        result = -EINVAL;
        goto err_find_node_failed;
    }

/* Get coefficient for color temperature calculation */
#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)
    /* Memory allocation */
    calc_color_parameter = kzalloc(sizeof(CALC_COLOR_PARAMETER), GFP_KERNEL);
    if (!calc_color_parameter)
    {
        BH1745_ERR( "Memory allocation failed.\n");
        result = -ENOMEM;
        goto err_color_memory_allocation_failed;
    }

    /* Read color temperature parameter from dts file */
    memset(node_name, 0, sizeof(node_name));
    sprintf(node_name, TP_COLOR_PARAMETER_PREFIX "%d", rgb->tp_module_id);
    BH1745_INFO("node name : %s\n", node_name);

    length = of_property_count_strings(node, node_name);
    BH1745_INFO("length = %d\n", length);
    BH1745_INFO("structure length = %lu\n", sizeof(CALC_COLOR_PARAMETER));

    if (length != sizeof(CALC_COLOR_PARAMETER)/TP_VALUE_LENGTH)
    {
        BH1745_ERR( "Parameter length does not match.Please fix!!!\n");
        result = -EINVAL;
        goto err_read_color_parameter_failed;
    }

    /* Read color temperature parameter one by one */
    for (index = 0;index < length;index++)
    {
        /* Read color temperature parameter base on index */
        result = of_property_read_string_index(node, node_name, index, &data);
        if (result)
        {
            BH1745_ERR( "Read color parameter from dts failed.\n");
            goto err_read_color_parameter_failed;
        }
        memcpy(calc_color_parameter->module_id[0] + index * TP_VALUE_LENGTH, data, TP_VALUE_LENGTH);
    }

    BH1745_WARNING("module id : %s\n", calc_color_parameter->module_id[0]);

    BH1745_WARNING("Golden judge:%s,\n\
        beff[0]     = %s, beff[1]     = %s, beff[2]     = %s, color[0]    = %s,\n\
        color[1]    = %s, color[2]    = %s, color[3]    = %s,\n\
        cct_beff[0] = %s, cct_beff[1] = %s, cct_beff[2] = %s, cct_beff[3] = %s\n",
        calc_color_parameter->gold_parameter.judge[0],  calc_color_parameter->gold_parameter.beff[0],
        calc_color_parameter->gold_parameter.beff[1],   calc_color_parameter->gold_parameter.beff[2],
        calc_color_parameter->gold_parameter.color[0],  calc_color_parameter->gold_parameter.color[1],
        calc_color_parameter->gold_parameter.color[2],  calc_color_parameter->gold_parameter.color[3],
        calc_color_parameter->gold_parameter.cct_eff[0],calc_color_parameter->gold_parameter.cct_eff[1],
        calc_color_parameter->gold_parameter.cct_eff[2],calc_color_parameter->gold_parameter.cct_eff[3]);

    BH1745_WARNING("white judge:%s,\n\
        beff[0]     = %s, beff[1]     = %s, beff[2]     = %s, color[0]    = %s,\n \
        color[1]    = %s, color[2]    = %s, color[3]    = %s,\n \
        cct_beff[0] = %s, cct_beff[1] = %s, cct_beff[2] = %s, cct_beff[3] = %s\n",
        calc_color_parameter->white_parameter.judge[0],  calc_color_parameter->white_parameter.beff[0],
        calc_color_parameter->white_parameter.beff[1],   calc_color_parameter->white_parameter.beff[2],
        calc_color_parameter->white_parameter.color[0],  calc_color_parameter->white_parameter.color[1],
        calc_color_parameter->white_parameter.color[2],  calc_color_parameter->white_parameter.color[3],
        calc_color_parameter->white_parameter.cct_eff[0],calc_color_parameter->white_parameter.cct_eff[1],
        calc_color_parameter->white_parameter.cct_eff[2],calc_color_parameter->white_parameter.cct_eff[3]);

    BH1745_WARNING("black judge:%s,\n\
        beff[0]     = %s, beff[1]     = %s, beff[2]     = %s, color[0] = %s,\n\
        color[1]    = %s, color[2]    = %s, color[3]    = %s,\n\
        cct_beff[0] = %s, cct_beff[1] = %s, cct_beff[2] = %s, cct_beff[3] = %s\n",
        calc_color_parameter->black_parameter.judge[0],  calc_color_parameter->black_parameter.beff[0],
        calc_color_parameter->black_parameter.beff[1],   calc_color_parameter->black_parameter.beff[2],
        calc_color_parameter->black_parameter.color[0],  calc_color_parameter->black_parameter.color[1],
        calc_color_parameter->black_parameter.color[2],  calc_color_parameter->black_parameter.color[3],
        calc_color_parameter->black_parameter.cct_eff[0],calc_color_parameter->black_parameter.cct_eff[1],
        calc_color_parameter->black_parameter.cct_eff[2],calc_color_parameter->black_parameter.cct_eff[3]);

    /* Get color_parameter accord to color type */
	if (GOLD == rgb->color_type)
    {
        color_coefficient_tmp = &calc_color_parameter->gold_parameter;
    }
    else if (WHITE == rgb->color_type)
    {
        color_coefficient_tmp = &calc_color_parameter->white_parameter;
    }
    else if (BLACK == rgb->color_type)
    {
        color_coefficient_tmp = &calc_color_parameter->black_parameter;
    }

    /* copy valid coefficent to rgb struct */
    memcpy(&rgb->ct_parameter, color_coefficient_tmp, sizeof(COLOR_PARAMETER));

    /* free calc_color_parameter */
    kfree(calc_color_parameter);
#endif

    return 0;

#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)
err_read_color_parameter_failed:
    kfree(calc_color_parameter);
err_color_memory_allocation_failed:
#endif
err_read_calc_lux_parameter_failed:
err_find_node_failed:
    kfree(calc_lux_parameter);

    return result;
}

/******************************************************************************
 * FUNCTION   : initialize system
 *****************************************************************************/
static int rgb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    RGB_DATA   *rgb;
    int         result;
	INIT_ARG 	 init_data;
    //struct als_control_path als_ctl = {0};
    //struct als_data_path    als_data = {0};

	BH1745_FUN();

    if (NULL == client || NULL == id){
        BH1745_ERR(" Parameter error \n");
        return EINVAL;
    }

	init_data.mode1_ctl = RGB_SET_MODE_CONTROL1;
	init_data.mode2_ctl = RGB_SET_MODE_CONTROL2;

    result = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
    if (!result) {
        BH1745_ERR( "need I2C_FUNC_I2C\n");
        result = -ENODEV;
        goto err_check_functionality_failed;
    }

    result = i2c_smbus_read_byte_data(client, REG_MANUFACT_ID);
    if (result < 0)
    {
        BH1745_ERR("Read data from IC error.\n");
        result = -EIO;
        goto err_check_functionality_failed;
    }

    if(result != MANUFACT_VALUE){
        BH1745_ERR("Error, IC value NOT correct!!! \n");
        result = -EINVAL;
        goto err_check_functionality_failed;
    }

	result = rgb_driver_init(client, init_data);
    if(result < 0){
        BH1745_ERR("rgb_driver_init fail\n");
        result = -EINVAL;
        goto err_check_functionality_failed;
    }

    rgb = kzalloc(sizeof(*rgb), GFP_KERNEL);
    if (rgb == NULL) {
        result = -ENOMEM;
        goto err_alloc_data_failed;
    }

    rgb->client = client;
    i2c_set_clientdata(client, rgb);

#if 0
    als_ctl.open_report_data = bh1745_open_report_data;
    als_ctl.enable_nodata = bh1745_enable_nodata;
    als_ctl.set_delay = bh1745_set_delay;
    als_ctl.is_use_common_factory = false;
    result = als_register_control_path(&als_ctl);
    if (result)
    {
        BH1745_ERR("als_register_control_path failed, error = %d\n", result);
        goto err_power_failed;
    }

    als_data.get_data = bh1745_get_data;
    als_data.vender_div = 1;
    result = als_register_data_path(&als_data);
    if (result)
    {
        BH1745_ERR("als_register_data_path failed, error = %d\n", result);
        goto err_power_failed;
    }
#endif

    /* Get the touch panel manufacture id and touch panel color type*/
    rgb->tp_module_id = bh1745_get_tp_module_id();
    rgb->color_type = bh1745_get_color_type();

    BH1745_ERR("Get parameters from device tree.\n");
    result = bh1745_get_parameter_from_dts(client->dev.of_node, rgb);
    if (result)
    {
        BH1745_ERR( "Get parameters from dts failed.\n");
        goto err_get_parameter_failed;
    }

#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)
    result = misc_register(&bh1745_misc);
    if (result)
    {
        BH1745_ERR( "Register misc device failed.\n");
        goto err_register_misc_device_failed;
    }
#endif

    obj = rgb;
	BH1745_ERR("rgb_i2c_probe OK.\n");
    return (result);

#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)
err_register_misc_device_failed:
#endif
err_get_parameter_failed:
err_power_failed:
    kfree(rgb);
err_alloc_data_failed:
err_check_functionality_failed:

    return (result);
}

/******************************************************************************
 * FUNCTION   : close system
 *****************************************************************************/
static int rgb_i2c_remove(struct i2c_client *client)
{
    RGB_DATA *rgb;
	BH1745_FUN();

    if (NULL == client){
        BH1745_ERR(" Parameter error \n");
        return EINVAL;
    }

    rgb = i2c_get_clientdata(client);
    kfree(rgb);
    return (0);
}

/******************************************************************************
 * FUNCTION   : initialize BH1745
 *****************************************************************************/
static int rgb_driver_init(struct i2c_client *client, INIT_ARG data)
{
    u8 w_mode[2];
    int result;
	BH1745_FUN();

    if (NULL == client){
        BH1745_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* execute software reset */
    result = rgb_driver_reset(client);
    if (result != 0) {
        return (result);
    }

    /* not check parameters are psth_upper, psth_low, alsth_upper, alsth_low */
    /* check the PS orerating mode */
    if (data.mode1_ctl > MEASUREMENT_MAX) {
        return (-EINVAL);
    }

    w_mode[0] = data.mode1_ctl;
    w_mode[1] = data.mode2_ctl;

    result = i2c_smbus_write_i2c_block_data(client, REG_MODECONTROL1, sizeof(w_mode), w_mode);
    if (result == 0) {
        result = i2c_smbus_write_byte_data(client, REG_MODECONTROL3, 0x02);
    }

    return (result);
}

/******************************************************************************
 * FUNCTION   : shutdown BH1745
 *****************************************************************************/
static int rgb_driver_shutdown(struct i2c_client *client)
{
    int result;
	BH1745_FUN();

    if (NULL == client){
        BH1745_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* set soft ware reset */
    result = rgb_driver_reset(client);

    return (result);
}

/******************************************************************************
 * FUNCTION   : reset BH1745 register
 *****************************************************************************/
static int rgb_driver_reset(struct i2c_client *client)
{
    int result;
	BH1745_FUN();

    if (NULL == client){
        BH1745_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* set soft ware reset */
    result = i2c_smbus_write_byte_data(client, REG_SYSTEMCONTROL, (SW_RESET | INT_RESET));

    return (result);
}

/******************************************************************************
 * FUNCTION   : power on and off BH1745
 *****************************************************************************/
static int rgb_driver_write_power_on_off(struct i2c_client *client, unsigned char data)
{
    int result;
    unsigned char mode_ctl2;
    unsigned char power_set;
    unsigned char write_data;	

    if (NULL == client){
        BH1745_ERR(" Parameter error \n");
        return EINVAL;
    }

    BH1745_WARNING(" data=%d\n", data);

    /* read mode control1 register */
    result = i2c_smbus_read_byte_data(client, REG_MODECONTROL2);
    if (result < 0) {
        return (result);
    }

    if (data == 0) {
        power_set = RGBC_EN_OFF;
    } 
	else {
        power_set = RGBC_EN_ON;
    }

    /* read mode control2 and mask RGBC_EN  */
    mode_ctl2  = (unsigned char)(result & ~RGBC_EN_ON);
    write_data = mode_ctl2 | power_set;
    result = i2c_smbus_write_byte_data(client, REG_MODECONTROL2, write_data);
    if (result < 0) {
        return (result);
    }

    /* delay 100ms, aaron add, 2016-1-2 */
    if(data){
        msleep(10);
    }
    return (result);
}

/******************************************************************************
 * FUNCTION : read the value of RGB data and status in BH1745
 *****************************************************************************/
static int rgb_driver_read_data(struct i2c_client *client, READ_DATA_ARG *data)
{
    int result;
    u8  read_data[8] = {0};

    if (NULL == client || NULL == data){
        BH1745_ERR(" Parameter error \n");
        return EINVAL;
    }

    /* block read */
    result = i2c_smbus_read_i2c_block_data(client, REG_READ_DATA, sizeof(read_data), read_data);
    if (result < 0) 
	{
        BH1745_ERR( "ps_rgb_driver_general_read : transfer error \n");
    } 
	else 
	{
        data->red   = read_data[0] | (read_data[1] << 8);
        data->green = read_data[2] | (read_data[3] << 8);
        data->blue  = read_data[4] | (read_data[5] << 8);
        data->clear = read_data[6] | (read_data[7] << 8);
        result      = 0;
    }

    return (result);
}

static int __init rgb_init(void)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	BH1745_ERR("i2c_number=%d\n",hw->i2c_num);
	bh1745_local_init();
  	//return alsps_driver_add(&bh1745_init_info);
}

static void __exit rgb_exit(void)
{
	BH1745_FUN();
	bh1745_local_uninit();
    return;
}

late_initcall(rgb_init);
module_exit(rgb_exit);

MODULE_DESCRIPTION("ROHM Ambient Light Sensor Driver");
MODULE_LICENSE("GPL");
