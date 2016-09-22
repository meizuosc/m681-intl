#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#include "kd_flashlight.h"
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>



/*
// flash current vs index
    0       1      2       3    4       5      6       7    8       9     10
93.74  140.63  187.5  281.25  375  468.75  562.5  656.25  750  843.75  937.5
     11    12       13      14       15    16
1031.25  1125  1218.75  1312.5  1406.25  1500mA
*/
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */
#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)


#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif



/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int gDuty;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);





static struct work_struct workTimeOut;

/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */

#ifdef GPIO_CAMERA_FLASH_EN_PIN
#define FLASH_GPIO_ENF GPIO_CAMERA_FLASH_EN_PIN
#else
#define FLASH_GPIO_ENF  (GPIO18|0X80000000)  // AL1518 modifed bu zhangdong
#endif

#ifdef GPIO_CAMERA_FLASH_MODE_PIN
#define FLASH_GPIO_ENT GPIO_CAMERA_FLASH_MODE_PIN
#else
#define FLASH_GPIO_ENT (GPIO16|0X80000000)   // AL1518 modified by zhangdong
#endif

#ifdef CONFIG_HQ_FLASHLIGHT_IC_LM3642   // AL1518 modified by zhangdong
#define HAVE_FLASHLIGHT_DRIVER_IC
#endif

static int gIsTorch[18] = { 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static int gLedDuty[18] = { 0, 32, 64, 96, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

/* current(mA) 50,94,141,188,281,375,469,563,656,750,844,938,1031,1125,1220,1313,1406,1500 */



/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

static struct i2c_client *RT4505_i2c_client;




struct RT4505_platform_data {
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

struct RT4505_chip_data {
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct RT4505_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

/* i2c access*/
/*
static int RT4505_read_reg(struct i2c_client *client, u8 reg,u8 *val)
{
	int ret;
	struct RT4505_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (ret < 0) {
		PK_ERR("failed reading at 0x%02x error %d\n",reg, ret);
		return ret;
	}
	*val = ret&0xff;

	return 0;
}*/

static int RT4505_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;
	struct RT4505_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_ERR("failed writting at 0x%02x\n", reg);
	return ret;
}

static int RT4505_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct RT4505_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}



/* ========================= */




static int RT4505_chip_init(struct RT4505_chip_data *chip)
{


	return 0;
}

static int RT4505_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct RT4505_chip_data *chip;
	struct RT4505_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("RT4505_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_ERR("RT4505 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct RT4505_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_ERR("RT4505 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct RT4505_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (RT4505_chip_init(chip) < 0)
		goto err_chip_init;

	RT4505_i2c_client = client;
	PK_DBG("RT4505 Initializing is done\n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_ERR("RT4505 probe is failed\n");
	return -ENODEV;
}

static int RT4505_remove(struct i2c_client *client)
{
	struct RT4505_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define RT4505_NAME "leds-RT4505"
static const struct i2c_device_id RT4505_id[] = {
	{RT4505_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id RT4505_of_match[] = {
	{.compatible = "mediatek,STROBE_MAIN"},
	{},
};
#endif

static struct i2c_driver RT4505_i2c_driver = {
	.driver = {
		   .name = RT4505_NAME,
#ifdef CONFIG_OF
		   .of_match_table = RT4505_of_match,
#endif
		   },
	.probe = RT4505_probe,
	.remove = RT4505_remove,
	.id_table = RT4505_id,
};

static int __init RT4505_init(void)
{
	PK_DBG("RT4505_init\n");
	return i2c_add_driver(&RT4505_i2c_driver);
}

static void __exit RT4505_exit(void)
{
	i2c_del_driver(&RT4505_i2c_driver);
}


module_init(RT4505_init);
module_exit(RT4505_exit);

MODULE_DESCRIPTION("Flash driver for RT4505");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

int readReg(int reg)
{

	int val;
	val = RT4505_read_reg(RT4505_i2c_client, reg);
	return (int)val;
}

/******************** LM3642 for AL1518 add start **********************/
#ifdef CONFIG_HQ_FLASHLIGHT_IC_LM3642
static struct i2c_client *LM3642_i2c_client = NULL;

#define I2C_STROBE_MAIN_SLAVE_7_BIT_ADDR  0x63
#define I2C_STROBE_MAIN_CHANNEL         1

struct LM3642_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct LM3642_chip_data {
	struct i2c_client *client;

	//struct led_classdev cdev_flash;
	//struct led_classdev cdev_torch;
	//struct led_classdev cdev_indicator;

	struct LM3642_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

static int LM3642_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	struct LM3642_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret =  i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_ERR("failed writting at 0x%02x\n", reg);
	return ret;
}

static int LM3642_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0;
	struct LM3642_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val =  i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}

static int LM3642_chip_init(struct LM3642_chip_data *chip)
{
	return 0;
}

static int LM3642_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct LM3642_chip_data *chip;
	struct LM3642_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("LM3642_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "LM3642 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3642_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL){ //values are set to Zero.
		PK_ERR("LM3642 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3642_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata  = pdata;
	if(LM3642_chip_init(chip)<0)
		goto err_chip_init;

	LM3642_i2c_client = client;
	PK_DBG("LM3642 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_ERR("LM3642 probe is failed \n");
	return -ENODEV;
}

static int LM3642_remove(struct i2c_client *client)
{
	struct LM3642_chip_data *chip = i2c_get_clientdata(client);

    if(chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}

#define LM3642_NAME "leds-LM3642"
static const struct i2c_device_id LM3642_id[] = {
	{LM3642_NAME, 0},
	{}
};

static struct i2c_driver LM3642_i2c_driver = {
	.driver = {
		.name  = LM3642_NAME,
	},
	.probe	= LM3642_probe,
	.remove   = LM3642_remove,
	.id_table = LM3642_id,
};

struct LM3642_platform_data LM3642_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_LM3642={ I2C_BOARD_INFO(LM3642_NAME, I2C_STROBE_MAIN_SLAVE_7_BIT_ADDR), \
													.platform_data = &LM3642_pdata,};

static int __init LM3642_init(void)
{
	printk("LM3642_init\n");
	int ret = 0;
	//i2c_register_board_info(2, &i2c_LM3642, 1);
	ret = i2c_register_board_info(I2C_STROBE_MAIN_CHANNEL, &i2c_LM3642, 1);
	if(ret < 0) {
		printk("%s: i2c_register_board_info error %d\n", __func__, ret);
		return ret;
	}

	ret = i2c_add_driver(&LM3642_i2c_driver);
	if(ret < 0 ) 
		printk("Failed to register SYG319 I2C driver: %d\n", ret);
	
	return ret; 
}

static void __exit LM3642_exit(void)
{
	i2c_del_driver(&LM3642_i2c_driver);
}


module_init(LM3642_init);
module_exit(LM3642_exit);

MODULE_DESCRIPTION("Flash driver for LM3642");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

#define LM3642_CON_REG1		(0xE0)
#define LM3642_CON_REG2		(0xF0)
#define LM3642_ENABLE_REG	(0x01)	//Enable register
#define LM3642_IVFM_REG		(0x02)
#define LM3642_LED1_FB_REG	(0x03)
#define LM3642_LED2_FB_REG	(0x04)
#define LM3642_LED1_TB_REG	(0x05)
#define LM3642_LED2_TB_REG	(0x06)

#define LM3642_FLASH1_REG	LM3642_LED2_FB_REG	/*******************************************/
#define LM3642_FLASH2_REG	LM3642_LED1_FB_REG	/***LM3643 LED1 ==> FLASH2(low temperature)***/
#define LM3642_TORCH1_REG	LM3642_LED2_TB_REG	/*******************************************/
#define LM3642_TORCH2_REG	LM3642_LED1_TB_REG	/***LM3643 LED2 ==> FLASH1(high temperature)***/
#define LM3642_TIMING_REG	(0x08)
#define FLASHIC_ID_REG		(0x0C)
/*
#define LM3642_FLASH_DUR_REG	(0xC0)
#define LM3642_FLASH_I_REG    	(0xB0)
#define LM3642_TORCH_I_REG	(0xA0)
#define LM3642_VIN_REG	(0x80)
*/
int m_duty1 = -1;
int m_duty2 = -1;
int LED1Closeflag = 1;
int LED2Closeflag = 1;

#define e_DutyNum	17
#define TORCHDUTYNUM	3
static int isMovieMode[e_DutyNum] = {1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int torchDuty[e_DutyNum] = {32,74,108,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//{17,35,53,71,89,106,0,0,0,0,0,0,0,0,0,0};// bit0 ~ 2
static int flashDuty[e_DutyNum] = {3,8,12,19,24,29,34,39,44,49,54,59,64,69,74,79,84};
						//{1,3,5,8,10,12,14,16,20,25,29,33,37,42,46,50,55,59,63,67,72,76,80,84,93,101,110,118,127};// bit0 ~ 3
static int torchDuty_T[e_DutyNum] = {17,35,53,71,89,106,0,0,0,0,0,0,0,0,0,0};

char *FLASH_IC;
int init_LM3642(void)
{
	int ret = 0, ic = -1;

	if(mt_set_gpio_mode(GPIO_FLASH_LED_EN,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    	if(mt_set_gpio_dir(GPIO_FLASH_LED_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    	if(mt_set_gpio_out(GPIO_FLASH_LED_EN,GPIO_OUT_ONE)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

	ret = LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x00);
	if(ret == 0)
		ret = LM3642_write_reg(LM3642_i2c_client, LM3642_TIMING_REG, 0x1F);

	LM3642_write_reg(LM3642_i2c_client, LM3642_LED1_TB_REG, 0x00);	//set reg 0x05(bit[7]) to 0 to resolve torch brightness code invalid sometimes
	LM3642_write_reg(LM3642_i2c_client, LM3642_LED1_FB_REG, 0x00);

	if (!FLASH_IC) {
		ic = LM3642_read_reg(LM3642_i2c_client, FLASHIC_ID_REG);
		if (ic == 0x02)
			FLASH_IC = "LM3643";
		else if (ic == 0x18)
			FLASH_IC = "SY7806";
	}
/*	else
		printk("flash ic not found \n");*/
	printk("FLASH_IC = %s \n", FLASH_IC);
/*	if(0 == ret)
		ret = LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x00);
	if(0 == ret)
		ret = LM3642_write_reg(LM3642_i2c_client, LM3642_FLASH_DUR_REG, 0x0B);  // Boost peak current:1.4A ; Flash timeout: 384 ms
	if(0 == ret)
		ret = LM3642_write_reg(LM3642_i2c_client, LM3642_VIN_REG, 0x3F);  // VIN enable, VIN flash enable, threshold 3.2V

    	if(mt_set_gpio_mode(FLASH_GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
   	if(mt_set_gpio_dir(FLASH_GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    	if(mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

    	if(mt_set_gpio_mode(FLASH_GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    	if(mt_set_gpio_dir(FLASH_GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    	if(mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
*/	
	return ret;
}

static int flashEnable_LM3642_led1(void)
{
	int temp;
	
#if 0
	temp = LM3642_read_reg(LM3642_i2c_client, LM3642_ENABLE_REG);

   	if(isMovieMode[m_duty1])
		LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, (temp & 0xF4) | 0x0A);  // LED1 torch mode
	else
		LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, (temp & 0xF4) | 0x0B);  // LED1 flash mode 	
#endif
	return 0;
}

static int flashDisable_LM3642_led1(void)
{
	int temp;
	
#if 0
	temp = LM3642_read_reg(LM3642_i2c_client, LM3642_ENABLE_REG);
	LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, temp | 0x10);  // LED1 close, 10000
#endif
	return 0;
}

static int setDuty_LM3642_led1(int duty)
{
	int ret = 0;
	int temp;	
	
	if(duty < 0)
		duty = 0;
	else if(duty >= e_DutyNum)
		duty = e_DutyNum - 1;

	m_duty1 = duty;
#if 0
	if(isMovieMode[duty]) {
		temp = LM3642_read_reg(LM3642_i2c_client, LM3642_TORCH_I_REG);
		ret = LM3642_write_reg(LM3642_i2c_client, LM3642_TORCH_I_REG, temp | torchDuty[duty]);
	}
	else {
		temp = LM3642_read_reg(LM3642_i2c_client, LM3642_FLASH_I_REG);
		ret = LM3642_write_reg(LM3642_i2c_client, LM3642_FLASH_I_REG, temp | flashDuty[duty]);
	}
#endif

	return ret;
}

int flashEnable_LM3642_led2(void)
{
	int temp;

#if 0	
	temp = LM3642_read_reg(LM3642_i2c_client, LM3642_ENABLE_REG);

   	if(isMovieMode[m_duty2])
		LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, (temp & 0xEC) | 0x12);  // LED2 torch mode
	else
		LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, (temp & 0xEC) | 0x13);  // LED2 flash mode 	
#else
	PK_DBG("flashEnable_LM3642_led2\n");
	PK_DBG("LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);

	//temp = LM3642_read_reg(LM3642_i2c_client, LM3642_ENABLE_REG);

	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
		LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x00);  // LED1 & LED2 close
	}
	else if(LED1Closeflag == 1)
	{
		if(isMovieMode[m_duty2] == 1)
			LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x09);//LED1(FLASH2) torch mode
		else
			LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x0D);//LED1(FLASH2) flash mode
	}
	else if(LED2Closeflag == 1)
	{
		if(isMovieMode[m_duty1] == 1)
			LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x0A);//LED2(FLASH1) torch mode
		else
			LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x0E);//LED2(FLASH1) flash mode
	}
	else
	{
		if((isMovieMode[m_duty1] == 1) && (isMovieMode[m_duty2] == 1))
			LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x0B);	// LED1&LED2 torch mode
		else
			LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x0F);	// LED1&LED2 flash mode
	}
#endif

	/*PK_DBG("0x01 = 0x%x, 0x02 = 0x%x, 0x03 = 0x%x, 0x04 = 0x%x, 0x05 = 0x%x,\
			0x06 = 0x%x, 0x07 = 0x%x, 0x08 = 0x%x, 0x0A = 0x%x, 0x0B = 0x%x \n",
			LM3642_read_reg(LM3642_i2c_client, 0x01),
			LM3642_read_reg(LM3642_i2c_client, 0x02),
			LM3642_read_reg(LM3642_i2c_client, 0x03),
			LM3642_read_reg(LM3642_i2c_client, 0x04),
			LM3642_read_reg(LM3642_i2c_client, 0x05),
			LM3642_read_reg(LM3642_i2c_client, 0x06),
			LM3642_read_reg(LM3642_i2c_client, 0x07),
			LM3642_read_reg(LM3642_i2c_client, 0x08),
			LM3642_read_reg(LM3642_i2c_client, 0x0A),
			LM3642_read_reg(LM3642_i2c_client, 0x0B));*/

	return 0;
}

int flashDisable_LM3642_led2(void)
{
	int temp;
#if 0	
	temp = LM3642_read_reg(LM3642_i2c_client, LM3642_ENABLE_REG);
	LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, temp | 0x08);  // LED1 close, 01000
#else
	flashEnable_LM3642_led2();
#endif
	return 0;
}

int setDuty_LM3642_led2(int duty)
{
	int ret = 0;
	int temp;

	if(duty < 0)
		duty = 0;
	else if(duty >= e_DutyNum)
		duty = e_DutyNum - 1;

	m_duty2 = duty;
#if 0
	if(isMovieMode[duty]) {
		temp = LM3642_read_reg(LM3642_i2c_client, LM3642_TORCH_I_REG);
		ret = LM3642_write_reg(LM3642_i2c_client, LM3642_TORCH_I_REG, temp | (torchDuty[duty] << 3));
	}
	else {
		temp = LM3642_read_reg(LM3642_i2c_client, LM3642_FLASH_I_REG);
		ret = LM3642_write_reg(LM3642_i2c_client, LM3642_FLASH_I_REG, temp | (flashDuty[duty] << 4));
	}
#else
	PK_DBG("setDuty_LM3642_led2:m_duty1 = %d, m_duty2 = %d!\n", m_duty1, m_duty2);
	PK_DBG("LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);

	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
		
	}
	else if(LED1Closeflag == 1)
	{
		if(isMovieMode[m_duty2] == 1)
		{
			ret = LM3642_write_reg(LM3642_i2c_client, LM3642_TORCH2_REG, torchDuty[m_duty2]);
		}
		else
		{
			ret = LM3642_write_reg(LM3642_i2c_client, LM3642_FLASH2_REG, flashDuty[m_duty2]);
		}
	}
	else if(LED2Closeflag == 1)
	{
		if(isMovieMode[m_duty1] == 1)
		{
			ret = LM3642_write_reg(LM3642_i2c_client, LM3642_TORCH1_REG, torchDuty[m_duty1]);
		}
		else
		{
			ret = LM3642_write_reg(LM3642_i2c_client, LM3642_FLASH1_REG, flashDuty[m_duty1]);
		}		
	}
	else
	{
		if((isMovieMode[m_duty1] == 1) && ((isMovieMode[m_duty2] == 1)))
		{
			ret = LM3642_write_reg(LM3642_i2c_client, LM3642_TORCH1_REG, torchDuty[m_duty1]);
			ret = LM3642_write_reg(LM3642_i2c_client, LM3642_TORCH2_REG, torchDuty[m_duty2]);
		}
		else
		{
			ret = LM3642_write_reg(LM3642_i2c_client, LM3642_FLASH1_REG, flashDuty[m_duty1]);
			ret = LM3642_write_reg(LM3642_i2c_client, LM3642_FLASH2_REG, flashDuty[m_duty2]);
		}
	}
#endif

	return ret;
}

static bool LM3642_flashlight_mode_inited = false;

int LM3642_torch_mode(int level)
{
	int ret;

	init_LM3642();	
	
	//ret = LM3642_write_reg(LM3642_i2c_client, LM3642_TORCH_I_REG, 0x05);  // set LED1 torch mode current 168.75mA

	if(0 == ret) {
		if(level > 0) {
			ret = LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x9B);  // enable LED1 torch mode, disable LED2
		} else {
			ret = LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x00);  // disable LED1, LED2
			if(!LM3642_flashlight_mode_inited)
    			mt_set_gpio_out(GPIO_FLASH_LED_EN, GPIO_OUT_ZERO);
		}
	}

	return ret;
}
EXPORT_SYMBOL(LM3642_torch_mode);

#endif
/******************** LM3642 for AL1518 add end **********************/

static int torch_inited = 0;

static int FL_Enable(void)
{
/*	int buf[2];
		buf[0] = 10;
	if (gIsTorch[gDuty] == 1)
		buf[1] = 0x71;
	else
		buf[1] = 0x77;
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
*/
	/*PK_DBG(" FL_Enable line=%d\n", __LINE__);*/
#ifndef HAVE_FLASHLIGHT_DRIVER_IC
	if(g_duty)
	{
		PK_DBG("\n FL_enable g_duty=%d\n",g_duty);
		mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ONE);
		mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ONE);
	} else if(0== g_duty) {
		PK_DBG("\n FL_endisable g_duty=%d\n",g_duty);

		mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
		mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ONE);	
	}
	return 0;
#else
	
#ifdef CONFIG_HQ_FLASHLIGHT_IC_LM3642
	//flashEnable_LM3642_led1();	 // open led1
	LM3642_torch_mode(1);
#endif
	PK_DBG(" FL_Enable line=%d\n",__LINE__);
	return 0;
#endif

}

static int FL_Disable(void)
{
/*	int buf[2];
	buf[0] = 10;
	buf[1] = 0x70;
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
*/
	/*PK_DBG(" FL_Disable line=%d\n", __LINE__);*/
#ifndef HAVE_FLASHLIGHT_DRIVER_IC
	mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
	mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO);
	return 0;
#else
#ifdef CONFIG_HQ_FLASHLIGHT_IC_LM3642
	//flashDisable_LM3642_led1();  // close led1
	LM3642_torch_mode(0);
#endif
	PK_DBG(" FL_Disable\n");
	return 0;
#endif
}

static int FL_dim_duty(int duty)
{
/*	int buf[2];
	if (duty > 17)
		duty = 17;
	if (duty < 0)
		duty = 0;
	gDuty = duty;
	buf[0] = 9;
	buf[1] = gLedDuty[duty];
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
*/
#ifndef HAVE_FLASHLIGHT_DRIVER_IC
	PK_DBG(" FL_dim_duty line=%d, duty=%d\n",__LINE__, duty);
	g_duty = duty;
	return 0;
#else
#ifdef CONFIG_HQ_FLASHLIGHT_IC_LM3642
	setDuty_LM3642_led1(duty);
#endif
	PK_DBG(" FL_dim_duty line=%d\n", __LINE__);
	return 0;
#endif;
}

static int FL_Init(void)
{
/*
	int buf[2];
	buf[0] = 0;
	buf[1] = 0x80;
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);

	buf[0] = 8;
	buf[1] = 0x7;
	RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
*/
#ifndef HAVE_FLASHLIGHT_DRIVER_IC
	if(mt_set_gpio_mode(FLASH_GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(FLASH_GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
	if(mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

	if(mt_set_gpio_mode(FLASH_GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(FLASH_GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
	if(mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

	PK_DBG(" FL_Init line=%d\n",__LINE__);
	return 0;
#else
#ifdef CONFIG_HQ_FLASHLIGHT_IC_LM3642
	init_LM3642();
	torch_inited = 1;
	LM3642_flashlight_mode_inited = true;
#endif
	PK_DBG(" FL_Init line=%d\n", __LINE__);
	return 0;
#endif
}


static int FL_Uninit(void)
{
	FL_Disable();

#ifdef CONFIG_HQ_FLASHLIGHT_IC_LM3642
	mt_set_gpio_out(GPIO_FLASH_LED_EN, GPIO_OUT_ZERO);
	torch_inited = 0;
	LM3642_flashlight_mode_inited = false;
#endif

	return 0;
}

int lm3643_torch_mode(mz_flash_data *torchData)
{
	int ret = 0;

	static int torch1_enabled = 0, torch2_enabled = 0;
	PK_DBG("flash1_level = %d, flash2_level = %d, flash1_enable = %d, flash2_enable = %d \n", \
			torchData->flash1_level, torchData->flash2_level, torchData->flash1_enable, torchData->flash2_enable);

	if (torch_inited == 0) {
		if (torchData->flash1_enable == 1 || torchData->flash2_enable == 1) {
			FL_Init();
			torch_inited = 1;
		} else {
			ret = LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x00);
			torch1_enabled = 0;
			torch2_enabled = 0;
			FL_Uninit();
			torch_inited = 0;
			return 0;
		}
	}

	LM3642_write_reg(LM3642_i2c_client, LM3642_LED1_TB_REG, torchDuty_T[torchData->flash2_level]);
	LM3642_write_reg(LM3642_i2c_client, LM3642_LED2_TB_REG, torchDuty_T[torchData->flash1_level]);

	PK_DBG("zhangdong LM3643_LED1_TORCH_CODE_REG = 0x%x, LM3643_LED2_TORCH_CODE_REG = 0x%x", \
								LM3642_read_reg(LM3642_i2c_client, LM3642_LED1_TB_REG), \
								LM3642_read_reg(LM3642_i2c_client, LM3642_LED2_TB_REG));

	if (torchData->flash1_enable == 1 && torchData->flash2_enable == 1) {
		ret = LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x3B);  // enable LED1 LED2 torch mode
	} else if (torchData->flash1_enable == 1) {
		ret = LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x3A);
	} else if (torchData->flash2_enable == 1) {
		ret = LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x39);
	} else {
		ret = LM3642_write_reg(LM3642_i2c_client, LM3642_ENABLE_REG, 0x30);
	}

	return ret;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
	/* printk(KERN_ALERT "work handler function./n"); */
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	static int init_flag;
	if (init_flag==0){
		init_flag=1;
	INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs=1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
	/* PK_DBG
	    ("RT4505 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg); */
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
#ifdef CONFIG_HQ_FLASHLIGHT_IC_LM3642
		m_duty1 = arg;
#endif
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;
				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
#ifdef CONFIG_HQ_FLASHLIGHT_IC_LM3642
			LED1Closeflag = 0;
#endif
			FL_Enable();
		} else {
#ifdef CONFIG_HQ_FLASHLIGHT_IC_LM3642
			LED1Closeflag = 1;
#endif
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	/*PK_DBG("constant_flashlight_open line=%d\n", __LINE__);*/
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_ERR(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	/*PK_DBG("constant_flashlight_open line=%d\n", __LINE__);*/

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
