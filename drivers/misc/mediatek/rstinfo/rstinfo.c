#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/hardirq.h>
#include <linux/init.h>
#include <linux/kallsyms.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/disp_assert_layer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/stacktrace.h>
#include <linux/compat.h>
#include <linux/aee.h>
#include <linux/seq_file.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/debugfs.h>  
#include <linux/module.h>  
#include <linux/mm.h>  
#include <linux/kthread.h>
#include <asm/uaccess.h> 
#include <linux/kmod.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/string.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/memory.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/namei.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/statfs.h>
#include <linux/stat.h>
#include <linux/dirent.h>
#include <linux/seq_file.h>


//#include <sys/stat.h>
static DECLARE_WAIT_QUEUE_HEAD(rstinfo_routine_wq);


//debug
#define RSTINFO_DEBUG 1
#if RSTINFO_DEBUG
#define rstinfo_log(fmt, arg...) printk("[rstinfo]" fmt, ##arg)
#else
#define rstinfo_log(fmt, arg...) 
#endif

#define EMMC_SUCCESS 0
#define EMMC_ERROR   -1
#define TRUE 1
#define FALSE 0

struct kobject *rstinfo_kobj;

#define RSTINFO_FILE      "/dev/block/platform/mtk-msdc.0/by-name/rstinfo"

#define RST_LOG_PATH      "/sdcard/kelog"

#define RST_LOG_FILE      "/sdcard/kelog/rst_log.txt"

#define KE_LOG_FILE1      "/sdcard/kelog/rst_ke1.txt"
#define KE_LOG_FILE2      "/sdcard/kelog/rst_ke2.txt"
#define KE_LOG_FILE3      "/sdcard/kelog/rst_ke3.txt"
#define KE_LOG_FILE4      "/sdcard/kelog/rst_ke4.txt"
#define KE_LOG_FILE5      "/sdcard/kelog/rst_ke5.txt"
#define KE_LOG_FILE6      "/sdcard/kelog/rst_ke6.txt"
#define KE_LOG_FILE7      "/sdcard/kelog/rst_ke7.txt"
#define KE_LOG_FILE8      "/sdcard/kelog/rst_ke8.txt"
#define KE_LOG_FILE9      "/sdcard/kelog/rst_ke9.txt"
#define KE_LOG_FILE10      "/sdcard/kelog/rst_ke10.txt"
#define KE_LOG_FILE11      "/sdcard/kelog/rst_ke11.txt"
#define KE_LOG_FILE12      "/sdcard/kelog/rst_ke12.txt"
#define KE_LOG_FILE13      "/sdcard/kelog/rst_ke13.txt"
#define KE_LOG_FILE14      "/sdcard/kelog/rst_ke14.txt"
#define KE_LOG_FILE15      "/sdcard/kelog/rst_ke15.txt"
#define KE_LOG_FILE16      "/sdcard/kelog/rst_ke16.txt"


#define LAST_KMSG_FILE      "/proc/last_kmsg"



#define BASE_ADDR  (0)
#define KERNEL_REBOOT_CNT_BASE_ADDR (2048)
#define ANDROID_REBOOT_CNT_BASE_ADDR (2048+1024*1024)
#define ANDROID_ROOT_CNT_BASE_ADDR (2048+2*1024*1024)
#define KE_LOG_BASE_ADDR (4*1024*1024)//4M-20M
#define LAST_KMSG_BUFFER_SIZE  (9*1024)


typedef enum {
	REBOOT_NORMAL_BOOT = 0,
	EXCEPTION_BOOT_PANIC,
    EXCEPTION_BOOT_HW_REBOOT,
    EXCEPTION_BOOT_HWT,
    EXCEPTION_BOOT_THERMAL_REBOOT,
    EXCEPTION_BOOT_OTHER_REBOOT
} rstinfo_boot_reason;

int g_lastreboot = REBOOT_NORMAL_BOOT;

unsigned int rstinfo_wdt_status = 0;
unsigned int rstinfo_fiq_status = 0;

int incrypt_flag = true;

int time_size = 31;//UTC time :%4d-%2d-%2d %2d:%2d:%2d \n     


struct KE_cnt_info{
	int ke_total_cnt;
	int panic_cnt;
	int hw_reboot_cnt;
	int wdt_reboot_cnt;
	int thermal_reboot_cnt;
	int other_reboot_cnt;
};//size 24

struct _head_info{
	char Head[8];
	struct KE_cnt_info ke_cnt;
	int android_rst_cnt;
	int android_root_cnt;
};

struct head_info{
	char Head[8];
	struct KE_cnt_info ke_cnt;
	int android_rst_cnt;
	int android_root_cnt;
	char reserve[2048-8-24-4-4];
};
struct header{
	struct head_info head;
	char ke_info [1024][1024];
	char android_reboot_info[1024][1024];
	char android_root_info[1024][1024];
	char reserve[4*1024*1024-3*1024*1024-2048];
};

struct rst_info{
	struct header header;
	char ke_log[16][1024*1024];
};


bool incrypt()
{
    return incrypt_flag;
}


bool folder_is_exit(const char *pathname)
{
	mm_segment_t old_fs;
	int fd;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fd = sys_open(pathname, O_RDONLY, 0);
	if (fd < 0) {
		//pr_err("%s: open folder %s failed.\n", TAG, pathname);
		set_fs(old_fs);
		return false;
	}
	set_fs(old_fs);
	sys_close(fd);
	return true;
}
int create_folder(const char *pathname)
{
	mm_segment_t old_fs;
	unsigned int file_mode, count = 0;
	long ret;

	file_mode = S_IALLUGO;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
retry:
	ret = sys_mkdirat(AT_FDCWD, pathname, file_mode);
	if (ret < 0 && count < 5 && ret != -EEXIST) {
		//pr_err("%s: mkdir %s error %ld.\n", TAG, pathname, ret);
		count++;
		goto retry;
	}
	set_fs(old_fs);
	return ret;
}


int strcmp_rst( char *str1,  char *str2)
{
		int delta = 0;
		int len,cnt=0;
		char last_char;
	    len = strlen(str1)-1;
		last_char = str1[len];
		str1[len]='\0';
		while (str1[cnt] ||str2[cnt]) {
			delta = str1[cnt] - str2[cnt];
			if (delta){
				str1[len] = last_char;
				return delta;
				}
			cnt++;
		}
		str1[len] = last_char;
		return 0;
}
int strcmp_reboot( char *str1,  char *str2)
	{
#if 0
			int delta = 0;
			int len,cnt=0;
			char last_char;
			len = strlen(str1)-1;
			last_char = str1[len];
			str1[len]='\0';
			while (str1[cnt] ||str2[cnt]) {
				delta = str1[cnt] - str2[cnt];
				if (delta){
					str1[len] = last_char;
					return delta;
					}
				cnt++;
			}
			str1[len] = last_char;
			return 0;
#endif
#if 1
		int delta = 0,i=0;
		int len,cnt=0;
		char last_char;
		len = strlen(str1)-1;
		last_char = str1[len];
		str1[len]='\0';
		if(len>7){
			while (str1[cnt] ||str2[cnt]) {
				delta = str1[cnt] - str2[cnt];
				if (delta){
					str1[len] = last_char;
					return delta;
					}
				cnt++;
				if((cnt == 6)&&(str1[cnt] == '_')){
					break;
				}
				
			}
			cnt++;
			i = len - cnt-1;
			for(;i>0;i--){
				printk("fxl cnt = %d,%c,%d\n",cnt,str1[cnt],len);
				if((str1[cnt]>='0')&&(str1[cnt]<='9')){
					cnt++;
				}else{
					cnt++;
					return -1;
				}
			}
			str1[len] = last_char;
			return 0;
			
		}else{
			while (str1[cnt] ||str2[cnt]) {
				delta = str1[cnt] - str2[cnt];
				if (delta){
					str1[len] = last_char;
					return delta;
					}
				cnt++;
			}
			str1[len] = last_char;
			return 0;
		}
#endif
	}



struct rtc_time rstinfo_get_current_time()
{
 struct timex  txc;
 struct rtc_time tm;
 do_gettimeofday(&(txc.time));
 rtc_time_to_tm(txc.time.tv_sec,&tm);
    return tm;
}




static int get_last_kmsg(int n,char * buf)
{
		struct file *f,*f_rst;
		mm_segment_t fs;
		int ret;
		unsigned long iSize;
		char buffer;
	    struct inode *inode = NULL;

		f = filp_open(LAST_KMSG_FILE, O_RDONLY, 0777);
		if (IS_ERR(f)) {
			printk("fxl read_mmc open file %s error!\n", LAST_KMSG_FILE);
			return -1;
		}
		fs = get_fs();set_fs(KERNEL_DS);
		f->f_pos=0;

		while(1){
				ret = f->f_op->read(f, &buffer, 1, &f->f_pos);//31 hws_status   43 fiq_step
				 //printk("fxlll read %c, %lld\n",buffer,f->f_pos);
					if(ret <= 0){
						buf[f->f_pos-1]='\0';
						break;
					}
					buf[f->f_pos-1]=buffer;
		}
		iSize = f->f_pos;
		set_fs(fs);
		filp_close(f, NULL);

	     printk("fxl read %ld, %s\n",iSize,buf);
		
		f_rst = filp_open(RSTINFO_FILE, O_RDWR, 0777);
		if (IS_ERR(f_rst)) {
			printk("fxl read_mmc open file %s error!\n", RSTINFO_FILE);
			return -1;
		}
		fs = get_fs();set_fs(KERNEL_DS);
		
		f_rst->f_pos=KE_LOG_BASE_ADDR+(n%16)*1024*1024;
		ret = f_rst->f_op->write(f_rst, buf, iSize, &f_rst->f_pos);
		set_fs(fs);
		filp_close(f_rst, NULL);

		if (ret < 0 ) {
			printk("fxl %s: read_mmc read file %s error!\n", __func__, LAST_KMSG_FILE);
			ret = -1;
		}
		return ret;
}



static int save_last_kmsg(int n,char * buf)
{
		struct file *f,*f_rst;
		mm_segment_t fs;
		int ret;
		int cnt;
		char buffer;
	    struct inode *inode = NULL;

		
		f_rst = filp_open(RSTINFO_FILE, O_RDONLY, 0777);
		if (IS_ERR(f_rst)) {
			printk("fxl read_mmc open file %s error!\n", RSTINFO_FILE);
			return -1;
		}
		fs = get_fs();set_fs(KERNEL_DS);
		f_rst->f_pos=KE_LOG_BASE_ADDR+(n%16)*1024*1024;
		printk("fxlfxl read f_rst->f_pos = %lld, n=%d\n",f_rst->f_pos,n);
		cnt = 0;
		while(1){
		ret = f_rst->f_op->read(f_rst, &buffer, 1, &f_rst->f_pos);
			if((ret <= 0)||(buffer=='\0')){
				printk("fxl return write_last_kmsg\n");
				buf[cnt++]='\0';
				break;
			}
			buf[cnt++]=buffer;
			
		}
		printk("fxl read_last_kmsg=%s\n",buf);
		set_fs(fs);
		filp_close(f_rst, NULL);

		if (ret < 0 ) {
			printk("fxl %s: read_mmc read file %s error!\n", __func__, LAST_KMSG_FILE);
			return -1;
		}
		if (folder_is_exit(RST_LOG_PATH) == false)
			create_folder(RST_LOG_PATH);

		n=(n%16)+1;
		switch (n) {
					case 1:
						f = filp_open(KE_LOG_FILE1, O_CREAT | O_RDWR, 0777);
						break;
					case 2:
						f = filp_open(KE_LOG_FILE2, O_CREAT | O_RDWR, 0777);
						break;
					case 3:
						f = filp_open(KE_LOG_FILE3, O_CREAT | O_RDWR, 0777);
						break;
					case 4:
						f = filp_open(KE_LOG_FILE4, O_CREAT | O_RDWR, 0777);
						break;
					case 5:
						f = filp_open(KE_LOG_FILE5, O_CREAT | O_RDWR, 0777);
						break;
					case 6:
						f = filp_open(KE_LOG_FILE6, O_CREAT | O_RDWR, 0777);
						break;
					case 7:
						f = filp_open(KE_LOG_FILE7, O_CREAT | O_RDWR, 0777);
						break;
					case 8:
						f = filp_open(KE_LOG_FILE8, O_CREAT | O_RDWR, 0777);
						break;
					case 9:
						f = filp_open(KE_LOG_FILE9, O_CREAT | O_RDWR, 0777);
						break;
					case 10:
						f = filp_open(KE_LOG_FILE10, O_CREAT | O_RDWR, 0777);
						break;
					case 11:
						f = filp_open(KE_LOG_FILE11, O_CREAT | O_RDWR, 0777);
						break;
					case 12:
						f = filp_open(KE_LOG_FILE12, O_CREAT | O_RDWR, 0777);
						break;
					case 13:
						f = filp_open(KE_LOG_FILE13, O_CREAT | O_RDWR, 0777);
						break;
					case 14:
						f = filp_open(KE_LOG_FILE14, O_CREAT | O_RDWR, 0777);
						break;
					case 15:
						f = filp_open(KE_LOG_FILE15, O_CREAT | O_RDWR, 0777);
						break;
					case 16:
						f = filp_open(KE_LOG_FILE16, O_CREAT | O_RDWR, 0777);
						break;

			}
    if (IS_ERR(f)) {
		printk("fxl %s: write_mmc open file %s error!\n", __func__, RSTINFO_FILE);
		return -1;
    }
	f->f_pos=0;
    fs = get_fs(); set_fs(KERNEL_DS);

    ret = f->f_op->write(f, buf, strlen(buf), &f->f_pos);
    set_fs(fs);
 
    filp_close(f, NULL);
    if ( ret < 0){
		printk("fxl write_mmc error ret =%d\n", ret);
    }
		return ret;
}


#if 0
static int read_emmc(int addr,char * buf,int len)
{
    struct file *f;
    mm_segment_t fs;
    int ret;

    f = filp_open(RSTINFO_FILE, O_RDONLY, 0777);
    if (IS_ERR(f)) {
        printk("fxl read_mmc open file %s error!\n", RSTINFO_FILE);
        return -1;
    }

    fs = get_fs(); set_fs(KERNEL_DS);
    f->f_pos=addr;
    ret = f->f_op->read(f, buf, len, &f->f_pos);
    set_fs(fs);
    filp_close(f, NULL);
    if (ret < 0 ) {
        printk("fxl %s: read_mmc read file %s error!\n", __func__, RSTINFO_FILE);
        ret = -1;
    }
	return ret;
}

static int write_emmc(int addr,char * buf, int len)
{
    struct file *f;
    mm_segment_t fs;
    int res;
    char *buffer = buf;

    f = filp_open(RSTINFO_FILE, O_RDWR, 0777);
    if (IS_ERR(f)) {
		printk("fxl %s: write_mmc open file %s error!\n", __func__, RSTINFO_FILE);
		return -1;
    }

    f->f_pos=addr;
    fs = get_fs(); set_fs(KERNEL_DS);

    res = f->f_op->write(f, buffer, len, &f->f_pos);
    set_fs(fs);
    filp_close(f, NULL);
    if ( res < 0){
		printk("fxl write_mmc error ret =%d\n", res);
    }
    return res;
}
#else
extern unsigned int emmc_read(char * part,unsigned int addr, unsigned int len,void *BufferPtr);
extern unsigned int emmc_write(char * part,unsigned int addr, unsigned int len,void *BufferPtr);

static int read_emmc_rstinfo(int addr,char * buf,int len)
{
    int ret,i;
    
   if (EMMC_SUCCESS !=emmc_read("rstinfo",addr,len,buf))
   {
   	printk("read_emmc_rstinfo error !\n");
	return EMMC_ERROR;
   }
    
    return EMMC_SUCCESS;
}

static int write_emmc_rstinfo(int addr,char * buf,int len)
{
    int ret,i;
    
   if (EMMC_SUCCESS !=emmc_write("rstinfo",addr,len,buf))
   {
   	printk("write_emmc_rstinfo error !\n");
	return EMMC_ERROR;
   }

    return EMMC_SUCCESS;
}

#endif




static int rstinfo_cnt_log_save(int addr,char * buf, int len)
{
    struct file *f;
    mm_segment_t fs;
    int res;
    char *buffer = buf;

	if (folder_is_exit(RST_LOG_PATH) == false)
				create_folder(RST_LOG_PATH);


    f = filp_open(RST_LOG_FILE, O_CREAT | O_RDWR, 0777);
    if (IS_ERR(f)) {
		printk("fxl %s: write_mmc open file %s error!\n", __func__, RSTINFO_FILE);
		return -1;
    }

    f->f_pos=addr;
    fs = get_fs(); set_fs(KERNEL_DS);

    res = f->f_op->write(f, buffer, len, &f->f_pos);
    set_fs(fs);
    filp_close(f, NULL);
    if ( res < 0){
		printk("fxl write_mmc error ret =%d\n", res);
    }
    return res;
}

int head_info_flag_init()
{
	
	char Head[8];
	int res;
	struct head_info head_info={"rstinfo"};
	char *head_info_temp = (char *)&head_info; 
	read_emmc_rstinfo(BASE_ADDR,Head,8);
	if(!strcmp(Head,"rstinfo")){
		printk("fxl head_info_flag exist\n");
		return 1;
	}else{
	    printk("fxl head_info_flag not exist\n");
		res = write_emmc_rstinfo(BASE_ADDR,head_info_temp,40);
		printk("fxl set head_info_flag OK!\n");
		return 0;
	}
}



void rstinfo_get_reboot_type()
{
	 struct file *f;
	 mm_segment_t fs;
	 int ret;
	 char buffer;
	 struct inode *inode = NULL;
	
	 f = filp_open(LAST_KMSG_FILE, O_RDONLY, 0777);
	 if (IS_ERR(f)) {
		 printk("fxl read_mmc open file %s error!\n", LAST_KMSG_FILE);
		 return -1;
	 }
	 fs = get_fs();set_fs(KERNEL_DS);
	 f->f_pos=31;
	 ret = f->f_op->read(f, &buffer, 1, &f->f_pos);//31 hws_status	 43 fiq_step
	 rstinfo_wdt_status	= buffer -48;	
	 f->f_pos=43;
	 ret = f->f_op->read(f, &buffer, 1, &f->f_pos);//31 hws_status	 43 fiq_step
	 rstinfo_fiq_status	= buffer -48;	
	 printk("fxl rstinfo_wdt_status %d, rstinfo_fiq_status %d\n",rstinfo_wdt_status,rstinfo_fiq_status);
	 set_fs(fs);
	 filp_close(f, NULL);



	if(((2==rstinfo_wdt_status)&&(0==rstinfo_fiq_status))||(0==rstinfo_wdt_status))
	{
	  g_lastreboot = REBOOT_NORMAL_BOOT;
	}
	else if((2==rstinfo_wdt_status)&&(0!=rstinfo_fiq_status))
	{
	  g_lastreboot = EXCEPTION_BOOT_PANIC;
	}
	else if(((1==rstinfo_wdt_status)||(5==rstinfo_wdt_status))&&(0==rstinfo_fiq_status))
	{
	   g_lastreboot = EXCEPTION_BOOT_HW_REBOOT;
	}
	else if(((1==rstinfo_wdt_status)||(5==rstinfo_wdt_status))&&(0!=rstinfo_fiq_status))
	{
		g_lastreboot = EXCEPTION_BOOT_HWT;
	}
	else if(0x32 == rstinfo_wdt_status)
	{
		g_lastreboot = EXCEPTION_BOOT_THERMAL_REBOOT;
	}
	else
	{
		g_lastreboot = EXCEPTION_BOOT_OTHER_REBOOT;
	}
}


int get_reboot_type()
{
	char *buff_temp=kmalloc(44,GFP_KERNEL);
	char *buffer = kmalloc(100*1024,GFP_KERNEL);//9k
	int log_addr;
	struct rtc_time tm;
	struct head_info head_info;
	char *head_info_temp = (char *)&head_info;
	memset(buff_temp,0,44);
	memset(buffer,0,100*1024);
	tm = rstinfo_get_current_time();//get time
    rstinfo_get_reboot_type();
	//return 1;
    if(g_lastreboot==REBOOT_NORMAL_BOOT){
					head_info_flag_init();
					read_emmc_rstinfo(BASE_ADDR,head_info_temp,40);
					switch (EXCEPTION_BOOT_PANIC) {
						case EXCEPTION_BOOT_PANIC:
							head_info.ke_cnt.panic_cnt++;
							break;
						case EXCEPTION_BOOT_HW_REBOOT:
							head_info.ke_cnt.hw_reboot_cnt++;
							break;
						case EXCEPTION_BOOT_HWT:
							head_info.ke_cnt.wdt_reboot_cnt++;
							break;
						case EXCEPTION_BOOT_THERMAL_REBOOT:
							head_info.ke_cnt.thermal_reboot_cnt++;
							break;
						case EXCEPTION_BOOT_OTHER_REBOOT:
							head_info.ke_cnt.other_reboot_cnt++;
							break;
		             }
		log_addr = KERNEL_REBOOT_CNT_BASE_ADDR +((head_info.ke_cnt.ke_total_cnt)%1000)*1024;
		head_info.ke_cnt.ke_total_cnt++;
		printk("fxl head_info.ke_cnt.ke_total_cnt =%d,head_info.ke_cnt.panic_cnt=%d\n",head_info.ke_cnt.ke_total_cnt,head_info.ke_cnt.panic_cnt);
		write_emmc_rstinfo(BASE_ADDR,head_info_temp,40);//cnt log
		//tm = rstinfo_get_current_time();//get time
		sprintf(buff_temp,"EXCEPTION %1d UTC time :%4d-%2d-%2d %2d:%2d:%2d \n",g_lastreboot,tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
		write_emmc_rstinfo(log_addr,buff_temp,strlen(buff_temp));//time log
	
		get_last_kmsg(--head_info.ke_cnt.ke_total_cnt,buffer);
	
		}else{
		 printk("fxl REBOOT_NORMAL_BOOT \n");
		}
	
}



static int rstinfo_head_check(struct head_info *head)
{
	if(strcmp(head->Head,"rstinfo")==0)
	{
		return TRUE;
	}
	return FALSE;
}


static int rstinfo_erase(void)
{
	struct _head_info head_info;
	memset(&head_info,0,sizeof(struct _head_info));
	
	if(EMMC_SUCCESS!=write_emmc_rstinfo(BASE_ADDR,&head_info,sizeof(struct _head_info)));//cnt log
	{
		
		rstinfo_log("rstinfo_erase fail!\n");
		return FALSE;
	}
	return TRUE;
}



static int rstinfo_incrypt(void)
{
	incrypt_flag = true;
	rstinfo_log("rstinfo_incrypt  incrypt_flag = %d \n",incrypt_flag);
}

static int rstinfo_decrypt(void)
{
	incrypt_flag = false;
	rstinfo_log("rstinfo_decrypt  incrypt_flag = %d \n",incrypt_flag);
}

static int rstinfo_check_password(int word)
{
	struct rtc_time tm;
	int password=0;
	tm = rstinfo_get_current_time();//get time
	password = tm.tm_min+tm.tm_sec/30;

	rstinfo_log("rstinfo_check_password password=%d,word=%d\n",password,word);
	if(word==password)
		return TRUE;
	return FALSE;
}

static int rstinfo_cnt_open(struct inode *inode, struct file *filp)   
{   
    filp->private_data = inode->i_private;  
    return 0;   
}   
    
static ssize_t rstinfo_cnt_read(struct file *filp, char __user *buffer,   
        size_t count, loff_t *ppos)   
{  
       int ret, cnt;
       struct _head_info head_info;
	
	/*if(incrypt()){
		printk("fxl rstinfo_cnt_log_read incrypt\n");
		return 0;
	}*/

	if(*ppos != 0){
		*ppos =0;
        return 0;
	}
	ret =read_emmc_rstinfo(BASE_ADDR,(char*)&head_info,sizeof(struct _head_info));
	if(ret!=0)
	{
		*ppos=sprintf(buffer,"rstinfo_cnt_read read emmc error!!!!!!!-----\n");
		return *ppos;
	}
	if(rstinfo_head_check(&head_info)!=TRUE)
	{
		memset(&head_info,0,40);
	}
	rstinfo_log("emmc read %s  count =%d\n",head_info.Head,(int)count);
	sprintf(buffer,"--------------------\n EXCEPTION REBOOT CNT = %d\n--------------------\n",head_info.ke_cnt.ke_total_cnt);
	sprintf(buffer,"%spanic_cnt REBOOT CNT = %d\n",buffer,head_info.ke_cnt.panic_cnt);
	sprintf(buffer,"%shw_reboot_cnt REBOOT CNT = %d\n",buffer,head_info.ke_cnt.hw_reboot_cnt);
	sprintf(buffer,"%swdt_reboot_cnt REBOOT CNT = %d\n",buffer,head_info.ke_cnt.wdt_reboot_cnt);
	sprintf(buffer,"%sthermal_reboot_cnt REBOOT CNT = %d\n",buffer,head_info.ke_cnt.thermal_reboot_cnt);
	sprintf(buffer,"%sother_reboot_cnt REBOOT CNT = %d\n--------------------\n",buffer,head_info.ke_cnt.other_reboot_cnt);

	*ppos = sprintf(buffer,"%s\n",buffer);
	return *ppos;

}   
    
static ssize_t rstinfo_cnt_write(struct file *filp, const char __user *buffer,  
        size_t count, loff_t *ppos)   
{   
	char info[10] ;
	int val;
	int incrypt;
	int ret=0;
	struct rtc_time tm;


	if (count >10)
	{
		rstinfo_log("rstinfo_cnt_write count>10!\n");
		return count;
	}
	memset(info,0,10);
	
	copy_from_user(info, buffer, count);

	if(0==strcmp(info,"erase\n"))
	{
	     rstinfo_erase();
	}else if(0==strcmp(info,"incrypt\n"))
	{
	     rstinfo_incrypt();
	}
	else if(sscanf(info,"%d",&val))
	{
		if(rstinfo_check_password(val)==TRUE)
		{
			rstinfo_decrypt();
		}
	}

	return count;



}   

static int rstinfo_cnt_log_open(struct inode *inode, struct file *filp)   
{  
    filp->private_data = inode->i_private;  
    return 0; 

}   
static ssize_t rstinfo_cnt_log_read(struct file *filp, char __user *buffer,   
        size_t count, loff_t *ppos)   
{  
	char buff_temp[44];
    int cnt;
	char cnt_end = '\0';
	int log_addr;

	memset(buff_temp,0,44);
	
    struct head_info head_info;
	char *head_info_temp = (char *)&head_info; 
	if(*ppos != 0){
		*ppos =0;
        return 0;
	}
	if(incrypt()){
		printk("fxl rstinfo_cnt_log_read incrypt\n");
		return 0;
	}

	
	read_emmc_rstinfo(BASE_ADDR,head_info_temp,40);
	sprintf(buffer,"--------------------\n EXCEPTION REBOOT CNT = %d\n--------------------\n",head_info.ke_cnt.ke_total_cnt);
	sprintf(buffer,"%spanic_cnt REBOOT CNT = %d\n",buffer,head_info.ke_cnt.panic_cnt);
	sprintf(buffer,"%shw_reboot_cnt REBOOT CNT = %d\n",buffer,head_info.ke_cnt.hw_reboot_cnt);
	sprintf(buffer,"%swdt_reboot_cnt REBOOT CNT = %d\n",buffer,head_info.ke_cnt.wdt_reboot_cnt);
	sprintf(buffer,"%sthermal_reboot_cnt REBOOT CNT = %d\n",buffer,head_info.ke_cnt.thermal_reboot_cnt);
	sprintf(buffer,"%sother_reboot_cnt REBOOT CNT = %d\n--------------------\n",buffer,head_info.ke_cnt.other_reboot_cnt);
	
	cnt = head_info.ke_cnt.ke_total_cnt;
	while(cnt--){
		log_addr = KERNEL_REBOOT_CNT_BASE_ADDR+cnt*1024;
		read_emmc_rstinfo(log_addr,buff_temp,44);	
		*ppos +=sprintf(buffer,"%s%s\n",buffer,buff_temp);
	}
	*ppos = sprintf(buffer,"%s%c\n",buffer,cnt_end);
	
	rstinfo_cnt_log_save(0,buffer,*ppos);
	return *ppos;

} 
static ssize_t rstinfo_cnt_log_write(struct file *filp, const char __user *buffer,  
        size_t count, loff_t *ppos)   
{  
	return 0;
} 


static int rstinfo_rst_log_open(struct inode *inode, struct file *filp)   
{    
    filp->private_data = inode->i_private;  
    return 0; 

}   
static ssize_t rstinfo_rst_log_read(struct file *filp, char __user *buffer,   
        size_t count, loff_t *ppos)   
{  
    int cnt,ret=-1;
    int log_addr;
	
    struct _head_info head_info;

	ret=read_emmc_rstinfo(BASE_ADDR,&head_info,40);
	if(ret!=0)
	{
		*ppos=sprintf(buffer,"rstinfo_rst_log_read read emmc error!!!!!!!-----\n");
		return *ppos;
	}
	if(head_info.ke_cnt.ke_total_cnt >=16){
		cnt = 16;
	}else{
		cnt = head_info.ke_cnt.ke_total_cnt;
	}
	while(cnt--){
		log_addr = KE_LOG_BASE_ADDR+(cnt%16)*1024*1024;
		//save_last_kmsg(cnt,buffer);	
	}
	return 0;

} 
static ssize_t rstinfo_rst_log_write(struct file *filp, const char __user *buffer,  
        size_t count, loff_t *ppos)   
{ 
	return 0;
} 


static int rstinfo_android_cnt_open(struct inode *inode, struct file *filp)   
{   
    filp->private_data = inode->i_private;  
    return 0; 

}   
static ssize_t rstinfo_android_cnt_read(struct file *filp, char __user *buffer,   
        size_t count, loff_t *ppos)   
{  
	char buff_temp[32];
       int cnt,ret=-1;
	char cnt_end = '\0';
	int log_addr;

	memset(buff_temp,0,32);
	
    struct _head_info head_info;

	if(*ppos != 0){
		*ppos =0;
        return 0;
	}

	ret=read_emmc_rstinfo(BASE_ADDR,&head_info,40);
	if(ret!=0)
	{
		*ppos=sprintf(buffer,"rstinfo_android_cnt_read read emmc error!!!!!!!-----\n");
		return *ppos;
	}
	sprintf(buffer,"--------------------\n REBOOT CNT = %d\n--------------------\n",head_info.android_rst_cnt);
	
	cnt = head_info.android_rst_cnt;
	while(cnt--){
		log_addr = ANDROID_REBOOT_CNT_BASE_ADDR+cnt*1024;
		read_emmc_rstinfo(log_addr,buff_temp,time_size);	
		*ppos +=sprintf(buffer,"%s%s\n",buffer,buff_temp);
	}
	*ppos = sprintf(buffer,"%s%c\n",buffer,cnt_end);
	return *ppos;

} 
static ssize_t rstinfo_android_cnt_write(struct file *filp, const char __user *buffer,  
        size_t count, loff_t *ppos)   
{  
	char info[12];
	char buff_temp[32];
	int res;
	int log_addr;
	struct rtc_time tm;
	struct _head_info head_info;
	memset(buff_temp,0,32);
	memset(info,0,12);
	
	copy_from_user(info, buffer, count);
	printk("fxl info =%d %ld %s\n",strcmp_reboot(info,"reboot"),strlen(info),info);
	if(!strcmp_reboot(info,"reboot")){		
		head_info_flag_init();
		read_emmc_rstinfo(BASE_ADDR,&head_info,40);
		log_addr = ANDROID_REBOOT_CNT_BASE_ADDR +((head_info.android_rst_cnt)%1000)*1024;
		head_info.android_rst_cnt=head_info.android_rst_cnt+1;
		write_emmc_rstinfo(BASE_ADDR,&head_info,40);//cnt log

		
		tm = rstinfo_get_current_time();//get time
		sprintf(buff_temp,"UTC time :%4d-%2d-%2d %2d:%2d:%2d \n",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
		write_emmc_rstinfo(log_addr,buff_temp,strlen(buff_temp));//time log	
	}else{
	
	    printk("fxl write rootinfo_cnt error!\n");
		
	}
	return count;


} 


static int rootinfo_cnt_open(struct inode *inode, struct file *filp)   
{     
    filp->private_data = inode->i_private;  
    return 0;   

} 


static ssize_t rootinfo_cnt_read(struct file *filp, char __user *buffer,   
        size_t count, loff_t *ppos)   
{  
	char buff_temp[32];
    int cnt;
	char cnt_end = '\0';
	int log_addr;
	int ret ;
	memset(buff_temp,0,32);
	
    struct _head_info head_info;
	if(*ppos != 0){
		*ppos =0;
        return 0;
	}
	ret = read_emmc_rstinfo(BASE_ADDR,&head_info,40);
	if((ret<0) || (head_info.android_root_cnt<=0)){
		printk("yili>>-------------------- ROOT CNT = %d--------------------\n",head_info.android_root_cnt);
		head_info.android_root_cnt = 0;
	}
	sprintf(buffer,"%d\n--------------------\n andriod root cnt: %d\n--------------------\n",head_info.android_root_cnt,head_info.android_root_cnt);
	
	cnt = head_info.android_root_cnt;
	while(cnt--){
		log_addr = ANDROID_ROOT_CNT_BASE_ADDR+cnt*1024;
		read_emmc_rstinfo(log_addr,buff_temp,time_size);	
		*ppos +=sprintf(buffer,"%s%s\n",buffer,buff_temp);
	}
	*ppos = sprintf(buffer,"%s%c\n",buffer,cnt_end);

	return *ppos;

} 
static ssize_t rootinfo_cnt_write(struct file *filp, const char __user *buffer,  
        size_t count, loff_t *ppos)   
{  
	char info[10];
	char buff_temp[32];
	int res;
	int log_addr;
	struct rtc_time tm;
	struct _head_info head_info;
	memset(buff_temp,0,32);
	memset(info,0,10);
	
	copy_from_user(info, buffer, count);
	if(!strcmp_rst(info,"root")){		
		head_info_flag_init();
		read_emmc_rstinfo(BASE_ADDR,&head_info,40);
		log_addr = ANDROID_ROOT_CNT_BASE_ADDR +((head_info.android_root_cnt)%1000)*1024;
		head_info.android_root_cnt=head_info.android_root_cnt+1;
		write_emmc_rstinfo(BASE_ADDR,&head_info,40);//cnt log

		
		tm = rstinfo_get_current_time();//get time
		sprintf(buff_temp,"UTC time :%4d-%2d-%2d %2d:%2d:%2d \n",tm.tm_year+1900,tm.tm_mon, tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
		write_emmc_rstinfo(log_addr,buff_temp,strlen(buff_temp));//time log	
	}else{
	
	    printk("fxl write rootinfo_cnt error!\n");
		
	}
	return count;

} 

	
struct file_operations rstinfo_cnt_fops = {   
    .owner = THIS_MODULE,  
    .open = rstinfo_cnt_open,  
    .read = rstinfo_cnt_read,  
    .write = rstinfo_cnt_write,  
};    
struct file_operations rstinfo_cnt_log_fops = {   
    .owner = THIS_MODULE,  
    .open = rstinfo_cnt_log_open,  
    .read = rstinfo_cnt_log_read,  
    .write = rstinfo_cnt_log_write,  
};  
struct file_operations rstinfo_rst_log_fops = {   
    .owner = THIS_MODULE,  
    .open = rstinfo_rst_log_open,  
    .read = rstinfo_rst_log_read,  
    .write = rstinfo_rst_log_write,  
};  
struct file_operations rstinfo_android_cnt_fops = {   
    .owner = THIS_MODULE,  
    .open = rstinfo_android_cnt_open,  
    .read = rstinfo_android_cnt_read,  
    .write = rstinfo_android_cnt_write,  
};  
struct file_operations rootinfo_cnt_fops = {   
    .owner = THIS_MODULE,  
    .open = rootinfo_cnt_open,  
    .read = rootinfo_cnt_read,  
    .write = rootinfo_cnt_write,  
}; 
static int rootinfo_test_open(struct inode *inode, struct file *filp)   
{     
    filp->private_data = inode->i_private;  
    return 0;   

} 
static ssize_t rootinfo_test_read(struct file *filp, char __user *buffer,   
        size_t count, loff_t *ppos)   
{ 
    
}
static ssize_t rootinfo_test_write(struct file *filp, const char __user *buffer,  
        size_t count, loff_t *ppos)   
{ 

{
		struct file *f,*f_rst;
		mm_segment_t fs;
		int ret;
		int iSize;
		char buffer;
	    struct inode *inode = NULL;


		
		f_rst = filp_open(RSTINFO_FILE, O_RDWR, 0777);
		if (IS_ERR(f_rst)) {
			printk("fxl read_mmc open file %s error!\n", RSTINFO_FILE);
			return -1;
		}
		fs = get_fs();set_fs(KERNEL_DS);
		
		ret = f->f_op->write(f, buffer, iSize, &f->f_pos);
		set_fs(fs);
		filp_close(f_rst, NULL);


		return 0;
}

}


struct file_operations rootinfo_test_fops = {   
    .owner = THIS_MODULE,  
    .open = rootinfo_test_open,  
    .read = rootinfo_test_read,  
    .write = rootinfo_test_write,  
};  

 static enum hrtimer_restart hrtimer_handler(struct hrtimer *timer)
 {

//  get_reboot_type();
 //kthread_run(get_reboot_type, NULL, "rstinfo_routine_thread");
 printk("fxl hrtimer_handler!\n");
 return HRTIMER_NORESTART;

 }

static int __init rstinfo_init(void)
{

 static struct hrtimer timer;
 ktime_t ktime;

 debugfs_create_file("rstinfo_cnt", 0666, NULL, NULL, &rstinfo_cnt_fops);  
 debugfs_create_file("rstinfo_cnt_log", 0666, NULL, NULL, &rstinfo_cnt_log_fops); 
 debugfs_create_file("rstinfo_rst_log", 0666, NULL, NULL, &rstinfo_rst_log_fops); 
 debugfs_create_file("rstinfo_android_cnt", 0666, NULL, NULL, &rstinfo_android_cnt_fops); 
 debugfs_create_file("rootinfo_cnt", 0666, NULL, NULL, &rootinfo_cnt_fops); 
// debugfs_create_file("rootinfo_test", 0777, NULL, NULL, &rootinfo_test_fops);


//  ktime = ktime_set(30, 0); /* 1 sec, 10 nsec */
//  hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
//  timer.function = hrtimer_handler;
//  hrtimer_start(&timer, ktime, HRTIMER_MODE_REL);


 
}
static void __exit rstinfo_exit(void)
{
   
}



module_init(rstinfo_init);
module_exit(rstinfo_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Huaqin rstinfo Driver");
MODULE_AUTHOR("Huaqin Inc.");
