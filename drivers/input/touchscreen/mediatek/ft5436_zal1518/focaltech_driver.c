/*
 * This software is licensed under the terms of the GNU General Public 
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms. 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * * VERSION      	DATE			AUTHOR          Note
 *    1.0		  2013-7-16			Focaltech        initial  based on MTK platform
 * 
 */
 
//////////////////////////////////////

#include "tpd.h"
#include <cust_alsps.h>

#include "tpd_custom_fts.h"
#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
#ifdef TPD_SYSFS_DEBUG
#include "focaltech_ex_fun.h"
#endif
#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#include "cust_gpio_usage.h"

//add by junpiao 20160203
#ifdef CONFIG_OF_TOUCH
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#endif


#include <linux/proc_fs.h>  /*proc*/
#include <linux/input/mt.h>  /*proc*/
//#include <mach/hardwareinfo.h>
//extern hardware_info_struct hardware_info;

//#define FTS_SCAP_TEST

//extern char tp_info[20];


//wangcq327 --- add start
static int TP_VOL;
//wangcq327 --- add end
  struct Upgrade_Info fts_updateinfo[] =
{
	{0x54,"FT5x46",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x54, 0x2c, 10, 2000},
	{0x36,"FT6x36",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x18, 10, 2000},//CHIP ID error
    {0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
    {0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x06, 100, 2000},
	{0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 1, 1500},
	{0x05,"FT6208",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,60, 30, 0x79, 0x05, 10, 2000},
	{0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
	{0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
	{0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
};
				
struct Upgrade_Info fts_updateinfo_curr;
static struct device *mx_tsp;   //add by wangyang for MX gesture
#ifdef TPD_PROXIMITY
#define APS_ERR(fmt,arg...)           	printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)

static u8 tpd_proximity_flag 			= 0;
static u8 tpd_proximity_suspend 		= 0;
static u8 tpd_proximity_detect 		= 1;//0-->close ; 1--> far away
static u8 tpd_proximity_detect_prev	= 0xff;//0-->close ; 1--> far away
#endif

#ifdef FTS_GESTRUE
extern int gesture_ps;
int gesture_onoff_fts = 0;

short pointnum = 0;

unsigned short coordinate_x[255] = {0};
unsigned short coordinate_y[255] = {0};
static int ft_wakeup_gesture = 0;
static long huawei_gesture = 0;
static int double_gesture = 0;
static int draw_gesture = 0;
static int gesture_echo[12]={0};
#define GT9XX_ECHO_PROC_FILE     "gesture_echo"

static ssize_t ft5436echo_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos);
static ssize_t ft5436echo_write_proc(struct file *, const char __user *, size_t, loff_t *);
/*
static struct proc_dir_entry *ft5436_echo_proc = NULL;

static const struct file_operations configecho_proc_ops = {
    .owner = THIS_MODULE,
    .read = ft5436echo_read_proc,
    .write = ft5436echo_write_proc,
};
*/
#endif
extern struct tpd_device *tpd;
 
static struct i2c_client *i2c_client = NULL;
struct i2c_client *fts_i2c_client = NULL;

struct task_struct *thread = NULL;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);

#ifdef CONFIG_OF_TOUCH
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc);
#else
static void tpd_eint_interrupt_handler(void);
#endif
//extern void mt_eint_unmask(unsigned int line);
//extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
//extern void mt_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);		
 
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
#ifdef USB_CHARGE_DETECT
static int b_usb_plugin = 0;
static int ctp_is_probe = 0; 
#endif

static int tpd_flag 					= 0;
static int tpd_halt						= 0;
static int point_num 					= 0;



//add by junpiao 20160203
#ifdef CONFIG_OF_TOUCH
unsigned int fts_touch_irq = 0;
#endif


int up_count=0;

//#define TPD_CLOSE_POWER_IN_SLEEP
#define TPD_OK 							0
//register define
#define DEVICE_MODE 					0x00
#define GEST_ID 						0x01
#define TD_STATUS 						0x02
//point1 info from 0x03~0x08
//point2 info from 0x09~0x0E
//point3 info from 0x0F~0x14
//point4 info from 0x15~0x1A
//point5 info from 0x1B~0x20
//register define

#define TPD_RESET_ISSUE_WORKAROUND

#define TPD_MAX_RESET_COUNT 			2

struct ts_event 
{
	u16 au16_x[CFG_MAX_TOUCH_POINTS];				/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];				/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];		/*touch event: 0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];			/*touch ID */
	u16 pressure[CFG_MAX_TOUCH_POINTS];
	u16 area[CFG_MAX_TOUCH_POINTS];
	u8 touch_point;
	int touchs;
	u8 touch_point_num;
};


#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#define VELOCITY_CUSTOM_FT5206
#ifdef VELOCITY_CUSTOM_FT5206
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

// for magnify velocity********************************************

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 			10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 			10
#endif

#define TOUCH_IOC_MAGIC 				'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)


static int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
static int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;


//add by wangyang for MX gesture
static u8 gesture_three_byte_one = 0;
static u8 gesture_three_byte_two = 0;
static u8 gesture_three_byte_three = 0;
static u8 gesture_three_byte_four = 0;
static int gesture_data=0;
static int gesture_ctrl=0;
static int gesture_mask = 0;
static int gesture_data1=0;
static int gesture_data2=0;
static int gesture_data3=0;
static ssize_t gesture_data_store(struct device* dev, struct device_attribute *attr,
                                  const char *buf, size_t count)
{  
/*
	struct alsps_context *cxt = NULL;
	//int err =0;
	ALSPS_LOG("als_store_active buf=%s\n",buf);
	mutex_lock(&alsps_context_obj->alsps_op_mutex);
	cxt = alsps_context_obj;

    if (!strncmp(buf, "1", 1)) 
	{
      	als_enable_data(1);
    	} 
	else if (!strncmp(buf, "0", 1))
	{
       als_enable_data(0);
    	}
	else
	{
	  ALSPS_ERR(" alsps_store_active error !!\n");
	}
	mutex_unlock(&alsps_context_obj->alsps_op_mutex);
	ALSPS_LOG(" alsps_store_active done\n");
    return count;
 */
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t gesture_data_show(struct device* dev, 
                                 struct device_attribute *attr, char *buf) 
{
/*
	struct alsps_context *cxt = NULL;
	int div = 0;
	cxt = alsps_context_obj;
	div=cxt->als_data.vender_div;
	ALSPS_LOG("als vender_div value: %d\n", div);
	return snprintf(buf, PAGE_SIZE, "%d\n", div); 
*/
	printk("gesture_data_show gesture_data=%x\n",gesture_data);
	return snprintf(buf, PAGE_SIZE, "%u\n",gesture_data);
}
/*----------------------------------------------------------------------------*/

static ssize_t gesture_control_node_store(struct device* dev, struct device_attribute *attr,
                                  const char *buf, size_t count)
{  
    u32 value,value1,value2;
    //sscanf(buf, "%x", &value);
    //gesture_ctrl = value;
    //value1 = value&0x000000ff;
    //value2 = (value>>16)&0x000000ff;
	int i = 0 ;
	char buff[4];
     //printk("yili: >> buf = %s\n",buf);
	/*
	the flyme upoper data flow:
	when gesture open,
	first data: x0x0		low bit -> hight bit
	end data  : 1010
	*/
	
	for(i=0;i<4;i++){
		buff[i]=*buf;
		buf++;
		}
    printk("[FTS] >> buff[0]= %d buff[1]= %d buff[2]= %d buff[3]= %d\n",buff[0],buff[1],buff[2],buff[3]);
    value1 = buff[0] ;
    value2 = buff[2] ; 

    if(value2 == 1)
    {
    	gesture_three_byte_one = value1;
		if(gesture_three_byte_one ==1){
			if((gesture_three_byte_two ==1)&&(gesture_three_byte_three ==255)&&(gesture_three_byte_four ==15))
				{gesture_mask = 64;}
			else if((gesture_three_byte_two ==0)&&(gesture_three_byte_three ==0)&&(gesture_three_byte_four ==0))
				{gesture_mask = 0;}
			else{
			gesture_mask = 128;}
			}
		else{
				gesture_mask = 0;
			}	

    }
    else if(value2 == 2)
    {
    	gesture_three_byte_two = value1;
		if(gesture_three_byte_one ==1){
			if((gesture_three_byte_two ==1)&&(gesture_three_byte_three ==255)&&(gesture_three_byte_four ==15))
				{gesture_mask = 64;}
			else if((gesture_three_byte_two ==0)&&(gesture_three_byte_three ==0)&&(gesture_three_byte_four ==0))
				{gesture_mask = 0;}
			else{
			gesture_mask = 128;}
			}
		else{
				gesture_mask = 0;
			}		
    }
    else if(value2 == 3)
    {
    	gesture_three_byte_three = value1;
		if(gesture_three_byte_one ==1){
			if((gesture_three_byte_two ==1)&&(gesture_three_byte_three ==255)&&(gesture_three_byte_four ==15))
				{gesture_mask = 64;}
			else if((gesture_three_byte_two ==0)&&(gesture_three_byte_three ==0)&&(gesture_three_byte_four ==0))
				{gesture_mask = 0;}
			else{
			gesture_mask = 128;}
			}
		else{
				gesture_mask = 0;
			}		
    }
    else if(value2 == 4)
    {
    	gesture_three_byte_four = value1;
		if(gesture_three_byte_one ==1){
			if((gesture_three_byte_two ==1)&&(gesture_three_byte_three ==255)&&(gesture_three_byte_four ==15))
				{gesture_mask = 64;}
			else if((gesture_three_byte_two ==0)&&(gesture_three_byte_three ==0)&&(gesture_three_byte_four ==0))
				{gesture_mask = 0;}
			else{
			gesture_mask = 128;}
			}
		else{
				gesture_mask = 0;
			}		
    }
	else
	{
		if(gesture_three_byte_one ==1){
			if((gesture_three_byte_two ==1)&&(gesture_three_byte_three ==255)&&(gesture_three_byte_four ==15))
				{gesture_mask = 64;}
			else if((gesture_three_byte_two ==0)&&(gesture_three_byte_three ==0)&&(gesture_three_byte_four ==0))
				{gesture_mask = 0;}
			else{
			gesture_mask = 128;}
			}
		else{
				gesture_mask = 0;
			}	
	}
	gesture_ctrl = gesture_mask;
    printk("[FTS]value = %x, value1 = %x, value2 = %x gesture_ctrl =gesture_mask = %x\n",value,value1,value2,gesture_mask);
    printk("[FTS]gesture_three_byte_one=%x,gesture_three_byte_two=%x,gesture_three_byte_three=%x,gesture_three_byte_four=%x\n",gesture_three_byte_one,gesture_three_byte_two,gesture_three_byte_three,gesture_three_byte_four);
    return count;
}

static ssize_t gesture_control_node_show(struct device* dev, 
                                 struct device_attribute *attr, char *buf) 
{
	return sprintf(buf,"%x\n",gesture_ctrl);
}
/*----------------------------------------------------------------------------*/

static DEVICE_ATTR(gesture_data,     		S_IWUSR | S_IRUGO, gesture_data_show, gesture_data_store);
static DEVICE_ATTR(gesture_control,      	S_IWUSR | S_IRUGO, gesture_control_node_show,  gesture_control_node_store);

static struct attribute *gesture_attributes[] = {
	//&dev_attr_gesture_control_node.attr,
	&dev_attr_gesture_data.attr,
	&dev_attr_gesture_control.attr,
	NULL
};

static struct attribute_group gesture_attribute_group = {
	.attrs = gesture_attributes
};
//add end


static int tpd_misc_open(struct inode *inode, struct file *file)
{
/*
	file->private_data = adxl345_i2c_client;

	if(file->private_data == NULL)
	{
		printk("tpd: null pointer!!\n");
		return -EINVAL;
	}
	*/
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	//file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int adxl345_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	//struct i2c_client *client = (struct i2c_client*)file->private_data;
	//struct adxl345_i2c_data *obj = (struct adxl345_i2c_data*)i2c_get_clientdata(client);	
	//char strbuf[256];
	void __user *data;
	
	long err = 0;
	
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		default:
			printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	return err;
}


static struct file_operations tpd_fops = {
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = TPD_NAME,
	.fops = &tpd_fops,
};

//**********************************************
#endif

struct touch_info {
    int y[10];
    int x[10];
    int p[10];//event
    int id[10];
    int pressure[10];
    int count;
    int eventnum;

};
 
static const struct i2c_device_id ft5206_tpd_id[] = {{TPD_NAME,0},{}};
//unsigned short force[] = {0,0x70,I2C_CLIENT_END,I2C_CLIENT_END}; 
//static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };
static struct i2c_board_info __initdata ft5206_i2c_tpd={ I2C_BOARD_INFO(TPD_NAME, (0x70>>1))};
 
static struct i2c_driver tpd_i2c_driver = {
  	.driver = {
	 	.name 	= TPD_NAME,
	//	.owner 	= THIS_MODULE,
  	},
  	.probe 		= tpd_probe,
  	.remove 	= tpd_remove,
  	.id_table 	= ft5206_tpd_id,
  	.detect 	= tpd_detect,
// 	.shutdown	= tpd_shutdown,
//  .address_data = &addr_data,
};

#ifdef USB_CHARGE_DETECT
void tpd_usb_plugin(int plugin)
{
	int ret = -1;
	b_usb_plugin = plugin;
	
	if(!ctp_is_probe)
	{
		return;
	}
	printk("Fts usb detect: %s %d %d.\n",__func__,b_usb_plugin,tpd_halt);
	if ( tpd_halt ) return;
	ret = i2c_smbus_write_i2c_block_data(i2c_client, 0x8B, 1, &b_usb_plugin);
	if ( ret < 0 )
	{
		printk("Fts usb detect write err: %s %d.\n",__func__,b_usb_plugin);
	}
}
EXPORT_SYMBOL(tpd_usb_plugin);
#endif




//*******************HQ  proc info*****************//
//#define HQ_DBG_CTP_INFO	//del junpiao for first debug 20160203
#ifdef HQ_DBG_CTP_INFO
extern int fts_get_fw_ver(struct i2c_client *client);

extern int fts_get_vendor_id(struct i2c_client *client);

static ssize_t proc_get_ctp_firmware_id(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
     char *page = NULL;
    char *ptr = NULL;
    int fw_ver,vid;
    int i, len, err = -1;

	  page = kmalloc(PAGE_SIZE, GFP_KERNEL);	
	  if (!page) 
	  {		
		kfree(page);		
		return -ENOMEM;	
	  }

    ptr = page; 
	if (*ppos)  // CMD call again
	{
		return 0;
	}
    /* Touch PID & VID */
    ptr += sprintf(ptr, "==== FT5436 Version ID ====\n");
    fw_ver=fts_get_fw_ver(i2c_client);
    vid=fts_get_vendor_id(i2c_client);
    ptr += sprintf(ptr, "Chip Vendor ID: 0x%x  FW_VERSION: 0x%x\n", vid,fw_ver);
    len = ptr - page; 			 	
    if(*ppos >= len)
	  {		
		  kfree(page); 		
		  return 0; 	
	  }	
	  err = copy_to_user(buffer,(char *)page,len); 			
	  *ppos += len; 	
	  if(err) 
	  {		
	    kfree(page); 		
		  return err; 	
	  }	
	  kfree(page); 	
	  return len;	
	
}

// used for gesture function switch
#ifdef FTS_GESTRUE
static ssize_t gesture_switch_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	char *ptr = page;
	int i;
	if (*ppos)  // CMD call again
		return 0;

	ptr += sprintf(ptr, "%d\n", ft_wakeup_gesture);

	*ppos += ptr - page;
	return (ptr - page);
}

static ssize_t gesture_switch_write_proc(struct file *file, char __user *buff, size_t size, loff_t *ppos)
{
	char wtire_data[32] = {0};

	if (size >= 32)
		return -EFAULT;

	if (copy_from_user( &wtire_data, buff, size ))
		return -EFAULT;

	if (wtire_data[0] == '1')
		ft_wakeup_gesture = 1;
	else
		ft_wakeup_gesture = 0;

	printk("zjy: %s, wakeup_gesture = %d\n", __func__, ft_wakeup_gesture);

	return size;
}
#if defined(FTS_HUAWEI_DESIGN)

static ssize_t ft5436echo_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	int len = 0;
	char *p = page;
	
    if (*ppos)  // CMD call again
    {
        return 0;
    }

	int i = 0;
	for(i=0;i<=11;i++)
	{
		p += sprintf(p, "%04x", gesture_echo[i]);
	}
	*ppos += p - page;

        memset(gesture_echo,0,sizeof(gesture_echo));

	return (p-page);
}

static ssize_t ft5436echo_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
 
}

static ssize_t huawei_gesture_switch_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	char *ptr = page;
	int i;
	if (*ppos)  // CMD call again
		return 0;

	ptr += sprintf(ptr, "%ld\n", huawei_gesture);

	*ppos += ptr - page;
	return (ptr - page);
}

static ssize_t huawei_gesture_switch_write_proc(struct file *file, char __user *buff, size_t size, loff_t *ppos)
{
	char wtire_data[32] = {0};

	if (size >= 32)
		return -EFAULT;

	if (copy_from_user( &wtire_data, buff, size ))
		return -EFAULT;
	huawei_gesture= simple_strtoul(wtire_data,NULL,10);

	if((huawei_gesture & 1) || (huawei_gesture & 0x2000)){
		ft_wakeup_gesture = 1;
	}else{
		ft_wakeup_gesture = 0;
	}
	
	if(huawei_gesture & 1)
		double_gesture = 1;
	else
		double_gesture = 0;

	if(huawei_gesture & 0x2000)
		draw_gesture = 1;
	else
		draw_gesture = 0;

	printk("zjh: %s, wangyang g_wakeup_gesture = %d huawei_gesture = %ld double_gesture = %d draw_gesture = %d\n", __func__, ft_wakeup_gesture,huawei_gesture,double_gesture,draw_gesture);
	return size;
}
#endif
#endif
#endif


static  void tpd_down(int x, int y,int pressure, int id) {

#ifdef MT_PROTOCOL_B
	input_mt_slot(tpd->dev, id);
	input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, 1);
#endif

	input_report_abs(tpd->dev, ABS_MT_PRESSURE, pressure);//0x3f//add  2
	//input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	//printk("%s x:%d y:%d p:%d\n", __func__, x, y, p);
	//input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, pressure);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	//input_mt_sync(tpd->dev);

#ifndef MT_PROTOCOL_B
	/* track id Start 0 */
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(tpd->dev);
#endif

/*	if(FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
	{
		printk("[FTS]tpd_down,tpd_button\n");
		tpd_button(x, y, 1);
	}*/

}
 
static  void tpd_up(int x, int y,int id) {

#ifdef MT_PROTOCOL_B
	input_mt_slot(tpd->dev, id);
	input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, 0);
#endif
	input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0);
	printk("%s x:%d y:%d id:%d\n", __func__, x, y,id);
	//input_report_key(tpd->dev, BTN_TOUCH, 0);
	//input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
#ifndef MT_PROTOCOL_B
	input_mt_sync(tpd->dev);
#endif

/*	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
	{
		printk("[FTS]tpd_up,tpd_button\n");
		tpd_button(x, y, 0);
	}*/

}

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;

	char data[128] = {0};
    	u16 high_byte,low_byte,reg;
	u8 report_rate =0;
	u8 pointid = 0x0f;
	u8 touch_point = 0x0f;

	if (tpd_halt)
	{
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}
	mutex_lock(&i2c_access);

       reg = 0x00;
	fts_i2c_Read(i2c_client, &reg, 1, data, POINT_READ_BUF);//64
	
	/////// add fts_monitor_record start
	fts_monitor_record(data, POINT_READ_BUF);
	/////// add fts_monitor_record end
	mutex_unlock(&i2c_access);
	
	if ((data[0] & 0x70) != 0)
		return false;
	
	memcpy(pinfo, cinfo, sizeof(struct touch_info));
	memset(cinfo, 0, sizeof(struct touch_info));
	/*get the number of the touch points*/
	cinfo->count = data[2] & 0x0f;
	cinfo->eventnum = 0;

	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++)
		cinfo->p[i] = 1;	/* Put up */
	
	for(i = 0; i < CFG_MAX_TOUCH_POINTS; i++)  
	{		
		cinfo->p[i] = (data[3 + 6 * i] >> 6) & 0x0003; /* event flag */
		cinfo->id[i] = data[3+6*i+2]>>4; 	// touch id
		if(cinfo->id[i] > CFG_MAX_TOUCH_POINTS)
			return true;
				
		/*get the X coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i];high_byte <<= 8;high_byte &= 0x0f00;
		
		low_byte = data[3 + 6 * i + 1];
		low_byte &= 0x00FF;
		cinfo->x[i] = high_byte | low_byte;
		
		/*get the Y coordinate, 2 bytes*/
		high_byte = data[3 + 6 * i + 2];high_byte <<= 8;high_byte &= 0x0f00;
		
		low_byte = data[3 + 6 * i + 3];
		low_byte &= 0x00FF;
		cinfo->y[i] = high_byte | low_byte;

		cinfo->pressure[i] = data[7 + 6 * i];
		
		cinfo->eventnum++;

	}
	
	return true;

}

#ifdef TPD_PROXIMITY
static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}

static int tpd_enable_ps(int enable)
{
	u8 state, state2;
	int ret = -1;

	i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
	printk("[proxi_5206]read: 999 0xb0's value is 0x%02X\n", state);
	if (enable){
		state |= 0x01;
		tpd_proximity_flag = 1;
		TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is on\n");	
	}else{
		state &= 0x00;	
		tpd_proximity_flag = 0;
		TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is off\n");
	}

	ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &state);
	TPD_PROXIMITY_DEBUG("[proxi_5206]write: 0xB0's value is 0x%02X\n", state);

	i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state2);
	if(state!=state2)
	{
		tpd_proximity_flag=0;
		printk("[proxi_5206]ps fail!!! state = 0x%x,  state2 =  0x%X\n", state,state2);
	}

	return 0;
}

static int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;

	hwm_sensor_data *sensor_data;
	TPD_DEBUG("[proxi_5206]command = 0x%02X\n", command);		
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value)
				{
					if((tpd_enable_ps(1) != 0))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						APS_ERR("disable ps fail: %d\n", err);
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{

				sensor_data = (hwm_sensor_data *)buff_out;

				sensor_data->values[0] = tpd_get_ps_value();
				TPD_PROXIMITY_DEBUG("huang sensor_data->values[0] 1082 = %d\n", sensor_data->values[0]);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}
#endif
 
#if 0 //def MT_PROTOCOL_B
/************************************************************************
* Name: fts_read_Touchdata
* Brief: report the point information
* Input: event info
* Output: get touch data in pinfo
* Return: success is zero
***********************************************************************/
static unsigned int buf_count_add=0;
static unsigned int buf_count_neg=0;
u8 buf_touch_data[30*POINT_READ_BUF] = { 0 };//0xFF
static int fts_read_Touchdata(struct ts_event *pinfo)
{
       u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FTS_MAX_ID;

	if (tpd_halt)
	{
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}

	mutex_lock(&i2c_access);
	ret = fts_i2c_Read(i2c_client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0)
	{
		dev_err(&i2c_client->dev, "%s read touchdata failed.\n",__func__);
		mutex_unlock(&i2c_access);
		return ret;
	}
	
	/////// add fts_monitor_record start
	fts_monitor_record(buf, POINT_READ_BUF);
	/////// add fts_monitor_record end
		
	mutex_unlock(&i2c_access);
	buf_count_add++;
	memcpy( buf_touch_data+(((buf_count_add-1)%30)*POINT_READ_BUF), buf, sizeof(u8)*POINT_READ_BUF );
	return 0;
}
static int fts_report_value(struct ts_event *data)
 {
	int i = 0,j = 0;
	int up_point = 0;
 	int touchs = 0;
	u8 pointid = FTS_MAX_ID;
	u8 buf[POINT_READ_BUF] = { 0 };//0xFF
	buf_count_neg++;
	memcpy( buf,buf_touch_data+(((buf_count_neg-1)%30)*POINT_READ_BUF), sizeof(u8)*POINT_READ_BUF );
	memset(data, 0, sizeof(struct ts_event));
	data->touch_point_num=buf[FT_TOUCH_POINT_NUM] & 0x0F;
	
	data->touch_point = 0;
	//printk("tpd  fts_updateinfo_curr.TPD_MAX_POINTS=%d fts_updateinfo_curr.chihID=%d \n", fts_updateinfo_curr.TPD_MAX_POINTS,fts_updateinfo_curr.CHIP_ID);
	for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++)
	{
		pointid = (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		else
			data->touch_point++;
		data->au16_x[i] =
		    (s16) (buf[FTS_TOUCH_X_H_POS + FTS_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FTS_TOUCH_X_L_POS + FTS_TOUCH_STEP * i];
		data->au16_y[i] =
		    (s16) (buf[FTS_TOUCH_Y_H_POS + FTS_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FTS_TOUCH_Y_L_POS + FTS_TOUCH_STEP * i];
		data->au8_touch_event[i] =
		    buf[FTS_TOUCH_EVENT_POS + FTS_TOUCH_STEP * i] >> 6;
		data->au8_finger_id[i] =
		    (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;
		data->pressure[i] =
			(buf[FTS_TOUCH_XY_POS + FTS_TOUCH_STEP * i]);//cannot constant value
		data->area[i] =
			(buf[FTS_TOUCH_MISC + FTS_TOUCH_STEP * i]) >> 4;
		if((data->au8_touch_event[i]==0 || data->au8_touch_event[i]==2)&&((data->touch_point_num==0)||(data->pressure[i]==0 && data->area[i]==0  )))
			return 1;



		//if ( pinfo->au16_x[i]==0 && pinfo->au16_y[i] ==0)
		//	pt00f++;
	}
	/*
	if (pt00f>0)
	{
		for(i=0;i<POINT_READ_BUF;i++)
		{
        		printk(KERN_WARNING "The xy00 is %x \n",buf[i]);
		}
		printk(KERN_WARNING "\n");
	}
	*/
	//event = data;
	for (i = 0; i < data->touch_point; i++)
	{
		 input_mt_slot(tpd->dev, data->au8_finger_id[i]);

		if (data->au8_touch_event[i]== 0 || data->au8_touch_event[i] == 2)
		{
			 input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,true);
			 input_report_abs(tpd->dev, ABS_MT_PRESSURE,data->pressure[i]/*0x3f*/);
			 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,data->area[i]/*0x05*/);
			 input_report_abs(tpd->dev, ABS_MT_POSITION_X,data->au16_x[i]);
			 input_report_abs(tpd->dev, ABS_MT_POSITION_Y,data->au16_y[i]);
			 touchs |= BIT(data->au8_finger_id[i]);
   			 data->touchs |= BIT(data->au8_finger_id[i]);
             		 //printk("tpd D x[%d] =%d,y[%d]= %d",i,data->au16_x[i],i,data->au16_y[i]);
		}
		else
		{
			 up_point++;
			 input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,false);
			 data->touchs &= ~BIT(data->au8_finger_id[i]);
		}

	}
 	if(unlikely(data->touchs ^ touchs)){
		for(i = 0; i < CFG_MAX_TOUCH_POINTS; i++){
			if(BIT(i) & (data->touchs ^ touchs)){
				input_mt_slot(tpd->dev, i);
				input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
			}
		}
	}
	data->touchs = touchs;
	//wangyang add begin 2015 12 30
	if (data->touch_point_num == 0)
	{
		// release all touches
		for(j = 0; j < CFG_MAX_TOUCH_POINTS; j++)
		{
			input_mt_slot( tpd->dev, j);
			input_mt_report_slot_state( tpd->dev, MT_TOOL_FINGER, false);
		}
		data->touchs = 0;
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_sync(tpd->dev);
		return;
	}
	//wangyang add end 2015 12 30
	if(data->touch_point == up_point)
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
	else
		 input_report_key(tpd->dev, BTN_TOUCH, 1);

	input_sync(tpd->dev);
	return 0;
    	//printk("tpd D x =%d,y= %d",event->au16_x[0],event->au16_y[0]);
 }

#endif


#ifdef FTS_GESTRUE
static void check_gesture(int gesture_id)
{
	

	 if((gesture_onoff_fts==1)&&(gesture_ps ==1)){
	 	gesture_id = 0x00 ;
		printk("[FTS]gesture_id==0x%x gesture_ps==%d gesture_onoff_fts==%d\n ",gesture_id,gesture_ps,gesture_onoff_fts);
	 }
	 
                if ((gesture_id == GESTURE_C) || (gesture_id == GESTURE_E) || (gesture_id == GESTURE_M) || 
					(gesture_id ==GESTURE_O) || (gesture_id == GESTURE_S) || (gesture_id == GESTURE_V) || 
                    (gesture_id == GESTURE_W) || (gesture_id == GESTURE_Z) 
                    )
                {
                	if ((((gesture_id == GESTURE_V) && ((gesture_three_byte_three & 0x01) == 0x01)) || 
			     ((gesture_id == GESTURE_C) && ((gesture_three_byte_three & 0x02) == 0x02)) || 
			     ((gesture_id == GESTURE_E) && ((gesture_three_byte_three & 0x04) == 0x04)) || 
			     ((gesture_id == GESTURE_W) && ((gesture_three_byte_three & 0x08) == 0x08)) || 
			     ((gesture_id == GESTURE_M) && ((gesture_three_byte_three & 0x10) == 0x10)) || 
			     ((gesture_id == GESTURE_S) && ((gesture_three_byte_three & 0x20) == 0x20)) || 
			     ((gesture_id == GESTURE_Z) && ((gesture_three_byte_three & 0x40) == 0x40)) ||
                 ((gesture_id == GESTURE_O) && ((gesture_three_byte_three & 0x80) == 0x80)) 
                    )&&(gesture_mask !=0))
                	{
                	printk("[FTS]:Wakeup by gesture(%c), light up the screen!", gesture_id);
                        input_report_key(tpd->dev, KEY_GESTURE, 1);
                        input_sync(tpd->dev);
                        input_report_key(tpd->dev, KEY_GESTURE, 0);
                        input_sync(tpd->dev);

                if(gesture_id == GESTURE_V)
			    {
			        gesture_data = 0xC7;
			    }
			    else if(gesture_id == GESTURE_C)
			    {
			        gesture_data = 0xC1;
			    }
			    else if(gesture_id == GESTURE_E)
			    {
			        gesture_data = 0xC0;
			    }
			    else if(gesture_id == GESTURE_W)
			    {
			        gesture_data = 0xC2;
			    }
			    else if(gesture_id == GESTURE_M)
			    {
			        gesture_data = 0xC3;
			    }
			    else if(gesture_id == GESTURE_S)
			    {
			        gesture_data = 0xC5;
			    }
			    else if(gesture_id == GESTURE_Z)
			    {
			        gesture_data = 0xCA;
			    }
			    else  //gesture_id == 'o'
			    {
			        gesture_data = 0xC4;
			    }
                        // clear 0x814B
                        gesture_id = 0x00;
                	}
			else
			{
                        gesture_id = 0x00;
			}
                }
                else if ( (gesture_id == 0x20) || (gesture_id == 0x21) ||(gesture_id == 0x22) || (gesture_id == 0x23) )
                {
			if((((gesture_id == 0x21) && ((gesture_three_byte_four & 0x01) == 0x01)) ||
				((gesture_id == 0x23) && ((gesture_three_byte_four & 0x04) == 0x04)) ||
				((gesture_id == 0x22) && ((gesture_three_byte_four & 0x08) == 0x08)) ||
				((gesture_id == 0x20) && ((gesture_three_byte_four & 0x02) ==0x02))
			)&&(gesture_mask !=0))
			{
                        printk("[FTS]: %x slide to light up the screen!", gesture_id);

                        input_report_key(tpd->dev, KEY_GESTURE, 1);
                        input_sync(tpd->dev);
                        input_report_key(tpd->dev, KEY_GESTURE, 0);
                        input_sync(tpd->dev);

			    if(gesture_id == 0x21)
			    {
			    	 gesture_data = 0xB1;
			    }
			    else if(gesture_id == 0x23)
			    {
			        gesture_data = 0xB3;
			    }
			    else if(gesture_id == 0x22)
			    {
			        gesture_data = 0xB2; 
			    }
			    else   //type==3
			    {
			        gesture_data = 0xB0;
			    }

			}
			else
			{
			    // clear 0x814B
                        gesture_id = 0x00;
			}
                }
                else if (0x24 == gesture_id)
                {
                	if(((gesture_three_byte_two & 0x01)== 0x01)&&(gesture_mask !=0))
                	{
                    	 printk("[FTS]:Double click to light up the screen!");

                   	 input_report_key(tpd->dev, KEY_GESTURE, 1);
                         input_sync(tpd->dev);
                         input_report_key(tpd->dev, KEY_GESTURE, 0);
                         input_sync(tpd->dev);
                         // clear 0x814B
                         gesture_id = 0x00;
						 
			    gesture_data = DOUBLE_TAP;
                	}
			else
			{
                        gesture_id = 0x00;
			}
                }
                else
                {
                    gesture_id = 0x00;
                }
            
            
        /*	
#if defined(FTS_HUAWEI_DESIGN)
	switch(gesture_id)
	{

		case GESTURE_DOUBLECLICK:
			if(double_gesture){
				input_report_key(tpd->dev, KEY_F1, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_F1, 0);
				input_sync(tpd->dev);
				}
			break;
		case GESTURE_C:
			if(draw_gesture){
				input_report_key(tpd->dev, KEY_F8, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_F8, 0);
				input_sync(tpd->dev);
				}
			break;
		case GESTURE_E:
			if(draw_gesture){
				input_report_key(tpd->dev, KEY_F9, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_F9, 0);
				input_sync(tpd->dev);
				}
			break;

		case GESTURE_M:
				if(draw_gesture){
				input_report_key(tpd->dev, KEY_F10, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_F10, 0);
				input_sync(tpd->dev);
				}
			break;
			
		case GESTURE_W:
				if(draw_gesture){
				input_report_key(tpd->dev, KEY_F11, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_F11, 0);
				input_sync(tpd->dev);
				}
			break;
		default:
			break;
	}

#else 
	switch(gesture_id)
	{

		case GESTURE_DOUBLECLICK:
				input_report_key(tpd->dev, KEY_F1, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_F1, 0);
				input_sync(tpd->dev);
			break;
		case GESTURE_C:
				input_report_key(tpd->dev, KEY_F8, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_F8, 0);
				input_sync(tpd->dev);
			break;

			
		case GESTURE_E:
				input_report_key(tpd->dev, KEY_F9, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_F9, 0);
				input_sync(tpd->dev);
			break;

		case GESTURE_M:
				input_report_key(tpd->dev, KEY_F10, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_F10, 0);
				input_sync(tpd->dev);
			break;
			
		case GESTURE_W:
				input_report_key(tpd->dev, KEY_F11, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_F11, 0);
				input_sync(tpd->dev);
			break;
		default:
			break;
	}
#endif
/*
		case GESTURE_W:
				input_report_key(tpd->dev, KEY_GESTURE_W, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_W, 0);
				input_sync(tpd->dev);
			break;

		case GESTURE_M:
				input_report_key(tpd->dev, KEY_GESTURE_M, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_V, 0);
				input_sync(tpd->dev);
			break;

		case GESTURE_S:
				input_report_key(tpd->dev, KEY_GESTURE_S, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_S, 0);
				input_sync(tpd->dev);
			break;

		case GESTURE_Z:
				input_report_key(tpd->dev, KEY_GESTURE_Z, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_Z, 0);
				input_sync(tpd->dev);
			break;
*/

}

static int ft5x0x_read_Touchdata(void)
{
        
    
		unsigned char buf[FTS_GESTRUE_POINTS * 4+2+6] = { 0 };
		//unsigned char buf[FTS_GESTRUE_POINTS * 2] = { 0 }; 
		int ret = -1;
		int i = 0;
		buf[0] = 0xd3;
		int gestrue_id = 0;
		short pointnum = 0;
		pointnum = 0;
		short feature_codes = 0x1111;
		u32 gesture_start_point_x,gesture_start_point_y;
		u32 gesture_end_point_x,gesture_end_point_y;
    		u32 gesture_p1_point_x,gesture_p1_point_y;
    		u32 gesture_p2_point_x,gesture_p2_point_y;
    		u32 gesture_p3_point_x,gesture_p3_point_y;
    		u32 gesture_p4_point_x,gesture_p4_point_y;
		ret = fts_i2c_Read(i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
		//ret = i2c_smbus_read_i2c_block_data(i2c_client, 0xd3, FTS_GESTRUE_POINTS_HEADER, buf);
		//i2c_smbus_write_i2c_block_data(i2c_client, 0xd3, 0, &buf);
		//i2c_smbus_read_i2c_block_data(i2c_client, 0xd3, FTS_GESTRUE_POINTS_HEADER, &buf);
		if (ret < 0)
		{
			printk( "%s read touchdata failed.\n", __func__);
			return ret;
		}
		//printk("ft5x0x_read_Touchdata buf[0]=%x \n",buf[0]);
		/* FW */

  		if(buf[0]!=0xfe)
    		{
      			  gestrue_id =  buf[0];
      			  //check_gesture(gestrue_id);
		}

    		pointnum = (short)(buf[1]) & 0xff;
    		buf[0] = 0xd3;
    		if((pointnum * 4 + 2 +36)<255) //  + 6
    		{
    			ret = fts_i2c_Read(i2c_client, buf, 1, buf, (pointnum * 4 + 2 + 36)); //  + 6
   		 }
    		else
   		 {
    			 ret = fts_i2c_Read(i2c_client, buf, 1, buf, 255);
         		 ret = fts_i2c_Read(i2c_client, buf, 0, buf+255, (pointnum * 4 + 2 + 36)-255); // + 6

    		  }
   		if (ret < 0)
    		{
        		printk( "%s read touchdata failed.\n", __func__);
        		return ret;
    		}
		for(i=0;i<18;i++)
		{
			feature_codes = (buf[pointnum * 4 + 2+i]<<8)|buf[pointnum * 4 + 2+1+i]; //  + 6
			printk("0x%X ", feature_codes);
		}  
			printk("0x%X  wangyang the pointnum = %d ", feature_codes,pointnum);
        		gesture_start_point_x   = (buf[pointnum * 4 + 2+4]<<8)|buf[pointnum * 4 + 2+1+4];
        		gesture_start_point_y  = (buf[pointnum * 4 + 2+6] << 8)|buf[pointnum * 4 + 2+1+6];
        		printk("gesture_start_point_x =0x%X, gesture_start_point_y =0x%X\n", gesture_start_point_x,gesture_start_point_y);
        		gesture_end_point_x   = (buf[pointnum * 4 + 2+8] << 8)|buf[pointnum * 4 + 2+1+8];
        		gesture_end_point_y  = (buf[pointnum * 4 + 2+10] << 8)|buf[pointnum * 4 + 2+1+10];
        		printk("gesture_end_point_x = 0x%X, gesture_end_point_y = 0x%X\n", gesture_end_point_x,gesture_end_point_y);
        		gesture_p1_point_x   = (buf[pointnum * 4 + 2+20] << 8)|buf[pointnum * 4 + 2+1+20];
        		gesture_p1_point_y   = (buf[pointnum * 4 + 2+22] << 8)|buf[pointnum * 4 + 2+1+22];
        		printk("gesture_p1_point_x = 0x%X, gesture_p1_point_y = 0x%X\n", gesture_p1_point_x,gesture_p1_point_y);
        		gesture_p2_point_x   = (buf[pointnum * 4 + 2+24] << 8)|buf[pointnum * 4 + 2+1+24];
        		gesture_p2_point_y   = (buf[pointnum * 4 + 2+26] << 8)|buf[pointnum * 4 + 2+1+26];
        		printk("gesture_p2_point_x = 0x%X, gesture_p2_point_y = 0x%X\n", gesture_p2_point_x,gesture_p2_point_y);
        		gesture_p3_point_x   = (buf[pointnum * 4 + 2+28] << 8)|buf[pointnum * 4 + 2+1+28];
        		gesture_p3_point_y   = (buf[pointnum * 4 + 2+30] << 8)|buf[pointnum * 4 + 2+1+30];
        		printk("gesture_p3_point_x = 0x%X, gesture_p3_point_y = 0x%X\n", gesture_p3_point_x,gesture_p3_point_y);
        		gesture_p4_point_x   = (buf[pointnum * 4 + 2+32] << 8)|buf[pointnum * 4 + 2+1+32];
        		gesture_p4_point_y   = (buf[pointnum * 4 + 2+34] << 8)|buf[pointnum * 4 + 2+1+34];
        		printk("gesture_p4_point_x = 0x%X, gesture_p4_point_y = 0x%X\n", gesture_p4_point_x,gesture_p4_point_y);
				gesture_echo[0]  = gesture_start_point_x;
				gesture_echo[1]  = gesture_start_point_y;	
				gesture_echo[2]  = gesture_end_point_x;
				gesture_echo[3]  = gesture_end_point_y;		
				gesture_echo[4]  = gesture_p1_point_x;
				gesture_echo[5] = gesture_p1_point_y;	
				gesture_echo[6] = gesture_p2_point_x;
				gesture_echo[7] = gesture_p2_point_y;	
				gesture_echo[8] = gesture_p3_point_x;
				gesture_echo[9] = gesture_p3_point_y;	
				gesture_echo[10] = gesture_p4_point_x;
				gesture_echo[11] = gesture_p4_point_y;	
				check_gesture(gestrue_id);
//add end
        	 return -1;
 }
	

#endif

//add by junpiao 20160203
#ifdef CONFIG_OF_TOUCH 
static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};
	node = of_find_compatible_node(NULL, NULL, "mediatek, TOUCH_PANEL-eint");
	if(node){
		of_property_read_u32_array(node , "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		fts_touch_irq = irq_of_parse_and_map(node, 0);		

		ret = request_irq(fts_touch_irq, tpd_eint_interrupt_handler,
					IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
		 if (ret > 0)
				TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
	} else {
		TPD_DMESG("[%s] tpd request_irq can not find touch eint device node!.", __func__);
	}
	return 0;
}
#endif

 static void fts_release_all_finger ( void )
 {
#ifdef MT_PROTOCOL_B
	 unsigned int finger_count=0;
#endif

#ifndef MT_PROTOCOL_B
	 input_mt_sync ( tpd->dev );
#else
	 for(finger_count = 0; finger_count < CFG_MAX_TOUCH_POINTS; finger_count++)
	 {
		 input_mt_slot( tpd->dev, finger_count);
		 input_mt_report_slot_state( tpd->dev, MT_TOOL_FINGER, false);
	 }
	 input_report_key(tpd->dev, BTN_TOUCH, 0);
#endif
	 input_sync ( tpd->dev );
 }

 static int touch_event_handler(void *unused)
{   

	struct touch_info cinfo, pinfo, ptest;
#ifdef MT_PROTOCOL_B
	struct ts_event pevent;
	int ret = 0;
#endif
	int i=0;

	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);

#ifdef TPD_PROXIMITY
	int err;
	hwm_sensor_data sensor_data;
	u8 proximity_status;
	
#endif
	u8 state;

	do
	{
//add by junpiao 20160203
#ifdef CONFIG_OF_TOUCH
	//enable_irq(fts_touch_irq);
#else
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif 
		set_current_state(TASK_INTERRUPTIBLE); 
	//	printk("wangcq327 --- waitting\n");
		wait_event_interruptible(waiter,tpd_flag!=0);				 
//		printk("wangcq327 --- pass\n");
		tpd_flag = 0;
 
		set_current_state(TASK_RUNNING);
		
#ifdef FTS_GESTRUE
		//i2c_smbus_read_i2c_block_data(i2c_client, 0xd0, 1, &state);
		fts_read_reg(i2c_client, 0xd0, &state);
//		if((get_suspend_state() == PM_SUSPEND_MEM) && (state ==1))
		if(state ==1)
		{
			ft5x0x_read_Touchdata();
			continue;
		}
#endif

#ifdef TPD_PROXIMITY
		if (tpd_proximity_flag == 1)
		{
			i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
			TPD_PROXIMITY_DEBUG("proxi_5206 0xB0 state value is 1131 0x%02X\n", state);

			if(!(state&0x01))
			{
				tpd_enable_ps(1);
			}

			i2c_smbus_read_i2c_block_data(i2c_client, 0x01, 1, &proximity_status);
			TPD_PROXIMITY_DEBUG("proxi_5206 0x01 value is 1139 0x%02X\n", proximity_status);

			if (proximity_status == 0xC0)
			{
				tpd_proximity_detect = 0;	
			}
			else if(proximity_status == 0xE0)
			{
				tpd_proximity_detect = 1;
			}

			TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);

			if(tpd_proximity_detect != tpd_proximity_detect_prev)
			{
				tpd_proximity_detect_prev = tpd_proximity_detect;
				sensor_data.values[0] = tpd_get_ps_value();
				sensor_data.value_divide = 1;
				sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
				if ((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
				{
					TPD_PROXIMITY_DMESG(" proxi_5206 call hwmsen_get_interrupt_data failed= %d\n", err);	
				}
			}
		}  
#endif
#if 0 //def MT_PROTOCOL_B
		{
        	ret = fts_read_Touchdata(&pevent);
			if (ret == 0)
				fts_report_value(&pevent);
		}
#else

		if (tpd_touchinfo(&cinfo, &pinfo)) 
		{
			if(cinfo.count > 0)
				input_report_key(tpd->dev, BTN_TOUCH, 1);
			
			for(i = 0; i < cinfo.eventnum; i++)  
			{
				if((0==cinfo.p[i]) || (2==cinfo.p[i]))
				{
		       		tpd_down(cinfo.x[i], cinfo.y[i], cinfo.pressure[i],cinfo.id[i]);
				}
				else if (1==cinfo.p[i])
				{
					tpd_up(cinfo.x[0], cinfo.y[0],cinfo.id[i]);
				}

				if(cinfo.p[i]==0)
					printk("[FTS]Down x:%d y:%d id:%d\n", cinfo.x[i], cinfo.y[i], cinfo.id[i]);
			}

			if(cinfo.count <= 0)
			{
#ifdef MT_PROTOCOL_B
				fts_release_all_finger();
#else
				input_report_key(tpd->dev, BTN_TOUCH, 0);
				tpd_up(0,0,0);
#endif
			}
			input_sync(tpd->dev);
		}
#endif		
	}while(!kthread_should_stop());

	return 0;
}
 
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
{
	strcpy(info->type, TPD_DEVICE);	
	return 0;
}

//add by junpiao 20160203
#ifdef CONFIG_OF_TOUCH
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
{
	TPD_DEBUG_PRINT_INT;
		
	tpd_flag = 1;
	/* enter EINT handler disable INT, make sure INT is disable when handle touch event including top/bottom half */
	/* use _nosync to avoid deadlock */
	//disable_irq_nosync(fts_touch_irq);
	wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}
#else
static void tpd_eint_interrupt_handler(void)
{
	//TPD_DEBUG("TPD interrupt has been triggered\n");
	TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;	
	wake_up_interruptible(&waiter);
}
#endif

void focaltech_get_upgrade_array(void)
{
	u8 chip_id;
	u32 i;

	i2c_smbus_read_i2c_block_data(i2c_client,FT_REG_CHIP_ID,1,&chip_id);
	DBG("%s chip_id = %x\n", __func__, chip_id);
	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
	{
		if(chip_id==fts_updateinfo[i].CHIP_ID)
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
			break;
		}
	}

	if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
	{
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
	}
}


extern int ctp_chip_id; //use for light sensor
extern int ctp_chip_colour;  //use for light sensor
extern unsigned char ft5x46_ctpm_InkId_get_from_boot(  struct i2c_client *client );	//use for lk_info ctp color port

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	 
	int retval = TPD_OK;
	char data;
	u8 report_rate=0;
	int err=0;
	int reset_count = 0;
	u8 chip_id,i,InkId;
#ifdef CONFIG_ARCH_MT6580
	int ret = 0;
#endif
	u8 uc_tp_fm_ver;
	u8  lsensor_ctp_chip_id; //use for light sensor

	client->timing = 100;

	i2c_client = client;
	fts_i2c_client = client;
	//power on, need confirm with SA
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(5);
	DBG(" fts ic reset\n");

//	DBG("wangcq327 --- %d\n",TPD_POWER_SOURCE_CUSTOM);	
#ifdef TPD_POWER_SOURCE_CUSTOM
#ifdef CONFIG_ARCH_MT6580
	tpd->reg = regulator_get(tpd->tpd_dev,TPD_POWER_SOURCE_CUSTOM); // get pointer to regulator structure
	if (IS_ERR(tpd->reg))
	{
		printk("focaltech tpd_probe regulator_get() failed!!!\n");
	}

	ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);	// set 2.8v
	if(ret)
	{
		printk("focaltech tpd_probe regulator_set_voltage() failed!\n");
	}
	ret = regulator_enable(tpd->reg);  //enable regulator
	if (ret)
	{
		printk("focaltech tpd_probe regulator_enable() failed!\n");
	}
#else
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#endif
#else
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif

#if 0 //def TPD_POWER_SOURCE_1800
	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif
	msleep(10);  //
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(5);


reset_proc:
#ifdef TPD_CLOSE_POWER_IN_SLEEP	 
	hwPowerDown(TPD_POWER_SOURCE,"TP");
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
	msleep(100);
#else
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(5);
	DBG(" fts ic reset\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(300);//400 sven
#endif

	//if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	err = i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data);
	DBG("gao_i2c:err %d,data:%d\n", err,data);
	if(err< 0 || data!=0)// reg0 data running state is 0; other state is not 0
	{
		DBG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
        if ( ++reset_count < TPD_MAX_RESET_COUNT )
        {
            goto reset_proc;
        }
#endif

#ifdef TPD_POWER_SOURCE_CUSTOM
#ifdef CONFIG_ARCH_MT6580
	ret	= regulator_disable(tpd->reg); //disable regulator
	if(ret)
	{
		printk("focaltech tpd_probe regulator_disable() failed!\n");
	}

	regulator_put(tpd->reg);
#else
		hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
#endif
#else
		hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
#endif
		return -1; 
	}

	msleep(10);//200 sven

//	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
//	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
//	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
//	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

#ifdef CONFIG_OF_TOUCH
        /* EINT device tree, default EINT enable */
		tpd_irq_registration();
#else
	//mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	//mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, tpd_eint_interrupt_handler, 1); 
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1); 
#endif

	tpd_load_status = 1;
 //   tpd_mstar_status =0 ;  // compatible mstar and ft6306 chenzhecong
 
	focaltech_get_upgrade_array();

#ifdef TPD_SYSFS_DEBUG
	fts_create_sysfs(i2c_client);
#ifdef FTS_APK_DEBUG
    		fts_create_apk_debug_channel(i2c_client);
    #endif
#endif

#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(i2c_client) < 0)
		DBG("%s:[FTS] create fts control iic driver failed\n",__func__);
#endif

	//add by wangyang for MX gesture
	mx_tsp=root_device_register("mx_tsp");
	err = sysfs_create_group(&mx_tsp->kobj,&gesture_attribute_group);
	if (err < 0)
	{
	   DBG("unable to create gesture attribute file\n");
	}
	//add end

/*
#ifdef FTS_HUAWEI_DESIGN
	ft5436_echo_proc = proc_create(GT9XX_ECHO_PROC_FILE, 0666, NULL, &configecho_proc_ops);
	if (ft5436_echo_proc == NULL)
	{
		printk("create_proc_entry %s failed\n", GT9XX_ECHO_PROC_FILE);
	}
#endif
*/

#ifdef VELOCITY_CUSTOM_FT5206
	if((err = misc_register(&tpd_misc_device)))
	{
		printk("mtk_tpd: tpd_misc_device register failed\n");
	}
#endif


#ifdef TPD_AUTO_UPGRADE
	printk("[FTS]********************Enter CTP Auto Upgrade********************\n");
	fts_ctpm_auto_upgrade(i2c_client);
#endif

	printk("[FTS]** Read chip_id CTP corlor CTP FW version********************\n");
	//for light sensor 
	fts_read_reg(i2c_client, FT_REG_CHIP_ID, &lsensor_ctp_chip_id);
	printk("[FTS] lsensor_ctp_chip_id = %x\n",lsensor_ctp_chip_id);
		ctp_chip_id=lsensor_ctp_chip_id;
		

	//for yeji ctp module(ic FT5436 ) : 0x11=black, 0x12 = white
	/******************** Read CTP corlor start********************/
	InkId = ft5x46_ctpm_InkId_get_from_boot(i2c_client);
	printk("[FTS] CTP corlor 0x11=black, 0x12 = white, inkid =0x%x\n",InkId);
		ctp_chip_colour = 1;
	/******************** Read CTP corlor end********************/

	//for hardware_info.tpd_fw_version = tpd->dev->id.version
	fts_read_reg(i2c_client, FTS_REG_FW_VER, &uc_tp_fm_ver);
	tpd->dev->id.version=(uc_tp_fm_ver&0xff);
	printk("[FTS] CTP FW version = 0x%x\n",tpd->dev->id.version);


	printk("[FTS]** touch_event_handler********************\n");

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread))
	{
		retval = PTR_ERR(thread);
		DBG(" failed to create kernel thread: %d\n", retval);
	}

//add by junpiao 20160203	
#ifdef CONFIG_OF_TOUCH
	enable_irq(fts_touch_irq);
#else
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif 

	DBG("FTS Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

#ifdef FTS_GESTRUE	
    input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
    input_set_capability(tpd->dev, EV_KEY, KEY_F1);
    input_set_capability(tpd->dev, EV_KEY, KEY_F2);
    input_set_capability(tpd->dev, EV_KEY, KEY_F3);
    input_set_capability(tpd->dev, EV_KEY, KEY_F4);
    input_set_capability(tpd->dev, EV_KEY, KEY_F5);
    input_set_capability(tpd->dev, EV_KEY, KEY_F6);
    input_set_capability(tpd->dev, EV_KEY, KEY_F7);
    input_set_capability(tpd->dev, EV_KEY, KEY_F8);
    input_set_capability(tpd->dev, EV_KEY, KEY_F9);
    input_set_capability(tpd->dev, EV_KEY, KEY_F10);
    input_set_capability(tpd->dev, EV_KEY, KEY_F11);
    input_set_capability(tpd->dev, EV_KEY, KEY_F12);
    input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE);

	__set_bit(KEY_F1, tpd->dev->keybit);
	__set_bit(KEY_F2, tpd->dev->keybit);
	__set_bit(KEY_F3, tpd->dev->keybit);
	__set_bit(KEY_F4, tpd->dev->keybit);
	__set_bit(KEY_F5, tpd->dev->keybit);
	__set_bit(KEY_F6, tpd->dev->keybit);
	__set_bit(KEY_F7, tpd->dev->keybit);
	__set_bit(KEY_F8, tpd->dev->keybit);
	__set_bit(KEY_F9, tpd->dev->keybit);
	__set_bit(KEY_F10, tpd->dev->keybit);
	__set_bit(KEY_F11, tpd->dev->keybit);
	__set_bit(KEY_F12, tpd->dev->keybit);
	__set_bit(KEY_GESTURE, tpd->dev->keybit);

	__set_bit(EV_KEY, tpd->dev->evbit);
	__set_bit(EV_SYN, tpd->dev->evbit);
#endif


#ifdef TPD_PROXIMITY
	struct hwmsen_object obj_ps;

	obj_ps.polling = 0;//interrupt mode
	obj_ps.sensor_operate = tpd_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		DBG("proxi_fts attach fail = %d\n", err);
	}
	else
	{
		DBG("proxi_fts attach ok = %d\n", err);
	}		
#endif

#ifdef MT_PROTOCOL_B
	#if 1//(LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))
		input_mt_init_slots(tpd->dev, MT_MAX_TOUCH_POINTS,1);
	#endif
		input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR,0, 255, 0, 0);
		input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, TPD_RES_X, 0, 0);
		input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, TPD_RES_Y, 0, 0);
		input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
#endif

#ifdef USB_CHARGE_DETECT  //meizu_usb_charge
		if(ctp_is_probe == 0)
		{
			 i2c_smbus_write_i2c_block_data(i2c_client, 0x8B, 1, &b_usb_plugin);
			ctp_is_probe =1;
		}
#endif

///////////////////init monitor module -start
#ifdef MT_PROTOCOL_B
		fts_monitor_init(tpd->dev, 1);
#else
		fts_monitor_init(tpd->dev, 0);
#endif
///////////////////init monitor module -end

#ifdef HQ_DBG_CTP_INFO
static struct hq_dbg_entry ctp_proc_entry[]=
   {
   		{
			.name="firmware_id",
			{	.owner = THIS_MODULE,
				.read=proc_get_ctp_firmware_id,
				.write=NULL,
			} 
		},
#ifdef FTS_GESTRUE
		{
		   .name="gesture_switch",	
	        {
			    .owner = THIS_MODULE,
			    .read  = gesture_switch_read_proc,
			    .write = gesture_switch_write_proc,
		  	}
		},
#if defined(FTS_HUAWEI_DESIGN)
	{
	   .name="g_huawei_control",	
          {
	    .owner = THIS_MODULE,
	    .read  = huawei_gesture_switch_read_proc,
	    .write = huawei_gesture_switch_write_proc,
	  }
	},
	{
	   .name="gesture_echo",	
          {
	    .owner = THIS_MODULE,
	    .read  = ft5436echo_read_proc,
	    .write = ft5436echo_write_proc,
	  }
	},
#endif
#endif
#ifdef HQ_GTP_GLOVE
		{
			.name = "glove_switch",
			{
				.owner = THIS_MODULE,
				.read  = glove_switch_read_proc,
				.write = glove_switch_write_proc,
			}
		},
#endif
	};
	//hq_gt9xx_client=client;
	//proc_hq_dbg_add("ctp", ctp_proc_entry, sizeof(ctp_proc_entry)/sizeof(struct hq_dbg_entry));
#endif	

	return 0;
 }

 static int tpd_remove(struct i2c_client *client)
{
	///add fts_monitor_exit
	fts_monitor_exit();

#ifdef FTS_APK_DEBUG
	fts_release_apk_debug_channel();
#endif
#ifdef TPD_SYSFS_DEBUG
	fts_release_sysfs(client);
#endif

#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
#endif

// zhuxiaming 0225
#ifdef TPD_POWER_SOURCE_CUSTOM
#ifdef CONFIG_ARCH_MT6580
	ret	= regulator_disable(tpd->reg); //disable regulator
	if(ret)
	{
		printk("focaltech tpd_probe regulator_disable() failed!\n");
	}

	regulator_put(tpd->reg);
#else
		hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
#endif
#else
		hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
#endif

#ifdef TPD_POWER_SOURCE_CUSTOM
#ifdef CONFIG_ARCH_MT6580
	regulator_disable(tpd->reg); //disable regulator
	regulator_put(tpd->reg);
#endif
#endif

	TPD_DEBUG("TPD removed\n");

   	return 0;
}
 
static int tpd_local_init(void)
{
  	DBG("FTS I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
 
   	if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
  		DBG("FTS unable to add i2c driver.\n");
      	return -1;
    }
    if(tpd_load_status == 0) 
    {
    	DBG("FTS add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
#ifndef MT_PROTOCOL_B
	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, (CFG_MAX_TOUCH_POINTS-1), 0, 0);
#endif

#ifdef TPD_HAVE_BUTTON     
	//if(TPD_RES_Y > 854)
	{
	    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
	}
	//else
	{
	    //tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local_fwvga);// initialize tpd button data
	}
#endif   

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
    DBG("end %s, %d\n", __FUNCTION__, __LINE__);  
    tpd_type_cap = 1;
    return 0; 
 }

static void tpd_resume( struct early_suspend *h )
{
  //int retval = TPD_OK;
  //char data;
  	int i;
#ifdef TPD_PROXIMITY	
	if (tpd_proximity_suspend == 0)
	{
		return;
	}
	else
	{
		tpd_proximity_suspend = 0;
	}
#endif	
 
   	DBG("TPD wake up\n");

#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
#else

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(5);  
   // mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
   // mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
	msleep(200);
	fts_release_all_finger();
	gesture_onoff_fts = 0;
//add by junpiao 20160203
#ifdef CONFIG_OF_TOUCH
	enable_irq(fts_touch_irq);
#else
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif 
	
	tpd_halt = 0;
	/* for resume debug
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("resume I2C transfer error, line: %d\n", __LINE__);
	}
	
	for(i = 0; i < CFG_MAX_TOUCH_POINTS; i++)
	{
		input_mt_slot(tpd->dev, i);
		input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, 0);
		DBG("zax TPD clear 1234.\n");
	}
	input_mt_report_pointer_emulation(tpd->dev, false);
	input_sync(tpd->dev);*/
	DBG("TPD wake up done\n");
#ifdef USB_CHARGE_DETECT
	msleep(120);
	tpd_usb_plugin(b_usb_plugin);
#endif
	 //return retval;
 }

 static void tpd_suspend( struct early_suspend *h )
 {
	// int retval = TPD_OK;
	 static char data = 0x3;
	u8 state = 0, state_g = 0;
	int i = 0;
/*		for (i = 0; i <CFG_MAX_TOUCH_POINTS; i++) 
	{
		input_mt_slot(tpd->dev, i);
		input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, 0);
		DBG("zax TPD enter sleep1234\n");
	}
	input_mt_report_pointer_emulation(tpd->dev, false);
	input_sync(tpd->dev);*/
	tpd_halt = 1;
	fts_release_all_finger();
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		tpd_proximity_suspend = 0;	
		return;
	}
	else
	{
		tpd_proximity_suspend = 1;
	}
#endif

#ifdef FTS_GESTRUE
  printk("[FTS] gesture_enable=:%d\n",ft_wakeup_gesture);
   if(((gesture_three_byte_one == 0) && ((gesture_three_byte_two == 0) && (gesture_three_byte_three == 0) && (gesture_three_byte_four == 0)))){
   }else{
     printk("[FTS] FTS_GESTRUE wakeup>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
   		gesture_onoff_fts = 1;
		memset(coordinate_x,0,255);
		memset(coordinate_y,0,255);
		
       	fts_write_reg(i2c_client, 0xd0, 0x01);
       // tpd_halt = 1;
		fts_write_reg(i2c_client, 0xd1, 0xff);				
		fts_write_reg(i2c_client, 0xd2, 0xff);
		fts_write_reg(i2c_client, 0xd5, 0xff);
		fts_write_reg(i2c_client, 0xd6, 0xff);
		fts_write_reg(i2c_client, 0xd7, 0xff);
		fts_write_reg(i2c_client, 0xd8, 0xff);
		msleep(10);
		
		for(i = 0; i < 10; i++)
		{
		printk("[FTS] tpd_suspend4 %d",i);
		   fts_read_reg(i2c_client, 0xd0, &state);
			if(state == 1)
			{
				TPD_DMESG("TPD gesture write 0x01\n");
        		return;
			}
			else
			{
				fts_write_reg(i2c_client, 0xd0, 0x01);
				fts_write_reg(i2c_client, 0xd1, 0xff);				
				fts_write_reg(i2c_client, 0xd2, 0xff);
				fts_write_reg(i2c_client, 0xd5, 0xff);
				fts_write_reg(i2c_client, 0xd6, 0xff);
				fts_write_reg(i2c_client, 0xd7, 0xff);
				fts_write_reg(i2c_client, 0xd8, 0xff);

				msleep(10);	
			  	fts_read_reg(i2c_client, 0xd0, &state);
				if(state == 1)
				{
		    		TPD_DMESG("TPD gesture write 0x01 again OK\n");
					return;
	   			}
				else
				{
					fts_write_reg(i2c_client, 0xd0, 0x01);
					msleep(10);
				}
			}
		}

			//i2c_smbus_read_i2c_block_data(i2c_client, 0xd0, 1, &state);
			//i2c_smbus_read_i2c_block_data(i2c_client, 0xd8, 1, &state_g);
			fts_read_reg(i2c_client, 0xd0, &state);
			fts_read_reg(i2c_client, 0xd8, &state_g);
			printk("[FTS]tpd fts_read_gesture ok data state=%d,d8 state_g= %d\n",state,state_g);
			return;
	}
#endif
#if 0//def MT_PROTOCOL_B //fts_release_all_finger
	 for (i = 0; i <CFG_MAX_TOUCH_POINTS; i++) 
	 {
		 input_mt_slot(tpd->dev, i);
		 input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, 0);
		 DBG("zax TPD enter sleep1234\n");
	 }
	 //input_mt_report_pointer_emulation(tpd->dev, false);
	 input_report_key(tpd->dev, BTN_TOUCH, 0);
	 input_sync(tpd->dev);
#endif

 	// tpd_halt = 1;

	DBG("TPD enter sleep\n");
//add by junpiao 20160203 
#ifdef CONFIG_OF_TOUCH
	disable_irq(fts_touch_irq);
#else
    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#endif

#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerDown(TPD_POWER_SOURCE,"TP");
#else
	mutex_lock(&i2c_access);
	i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
	mutex_unlock(&i2c_access);
#endif
	DBG("TPD enter sleep done\n");
	//return retval;
 } 


 static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = TPD_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif		
 };
 /* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	printk("MediaTek FTS touch panel driver init\n");
	i2c_register_board_info(IIC_PORT, &ft5206_i2c_tpd, 1);
//	i2c_register_board_info(0, &ft5206_i2c_tpd, 1);
	if(tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add FTS driver failed\n");
	 return 0;
 }
 
 /* should never be called */
static void __exit tpd_driver_exit(void) {
	TPD_DMESG("MediaTek FTS touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
	sysfs_remove_group(&mx_tsp->kobj,&gesture_attribute_group);
        root_device_unregister(mx_tsp);
}
 
module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
