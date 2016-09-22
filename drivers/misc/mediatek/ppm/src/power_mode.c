/*
 * kernel/power/power_mode.c
 *
 * add /sys/power/power_mode for set save mode mode .
 *
 * Copyright (C) 2014 chenshb. chenshb <chenshb@meizu.com>
 */

#include <linux/device.h>
#include <linux/mutex.h>
//#include "power.h"

//#include <linux/power_mode.h>
#include "mt_ppm_internal.h"
#include "power_mode.h"

static DEFINE_MUTEX(power_lock);


static int power_mode_debug;
struct power_mode_info *cur_power_mode;
unsigned int cpu_power_mode;

struct ppm_cluster_limit cur_ppm_limit[2]={-1};
int cur_gpu_freq_idx=0;



/* other members will init by ppm_main */
static struct ppm_policy_data user_mode_policy = {
	.name			= __stringify(PPM_POLICY_USER_MODE),
	.policy			= PPM_POLICY_USER_MODE,
	.priority		= PPM_POLICY_PRIO_HIGHEST,
	.get_power_state_cb	= NULL,
	.update_limit_cb	= NULL,
	.status_change_cb	= NULL,
	.mode_change_cb		= NULL,
};

/*
little_freq: 1300000 1235000 1144000 1040000 819000 598000 442000 299000
gpu_freq: 448500 299000
*/

int cpu_freq_table[2][8]=
{
	{
		1001000,
		910000,
		819000,
		689000,
		598000,
		494000,
		338000,
		156000,
	},
	{
		1807000,
		1651000,
		1495000,
		1196000,
		1027000,
		871000,
		663000,
		286000,
	}

};
int gpu_freq_table[6]=
{
	546000,
	520000,
	468000,
	429000,
	390000,
	351000,
};

#if 0
struct power_mode_info power_mode_info_tlb[POWER_MODE_END] = {
	/* name, big_freq, little_freq, gpu_freq, little_num, big_num, little_turbo, big_turbo */
	{ "low",   0, 1235000, 299000, 4, 0, 0, 0, 60},
	{ "normal",0, 1300000, 448500, 6, 0, 1, 0, 65},
	{ "high",  0, 1300000, 448500, 8, 0, 1, 0, 70},
	{ "custom",  0, 1300000, 448500, 8, 0, 1, 0, 70},
};
#else
struct power_mode_info power_mode_info_tlb[POWER_MODE_END] = {
	/* name, big_freq, little_freq, gpu_freq, little_num, big_num, little_turbo, big_turbo */
	{ "low",      1001000,  156000,  1495000,   286000,   4,  1,  0,  0,  546000},
	{ "normal", 1001000,  156000,  1807000,   286000,   2,  0,  4,  0,  546000},
	{ "high",     1001000,  156000,  1807000,   286000,   4,  0,  4,  0,  546000},
};
#endif
static struct pm_qos_request little_cpu_max_freq_qos;
static struct pm_qos_request little_cpu_max_num_qos;
static struct pm_qos_request gpu_max_freq_qos;







int find_cpu_freq_idx(int cluster_num,int freq)
{
	int i=0;
	for (i=0;i<8;i++)
	{
		//printk("find_cpu_freq_idx : %d   %d   %d",cluster_num ,cpu_freq_table[cluster_num][i],freq);
		if(cpu_freq_table[cluster_num][i]==freq)
			return i;
	}
	return -1;
}

int find_gpu_freq_idx(int freq)
{
	int i=0;
	for (i=0;i<6;i++)
	{
		if(gpu_freq_table[i]==freq)
			return i;
	}
	return -1;
}

int set_cur_power_mode(struct power_mode_info *power_mode)
{
	
	cur_ppm_limit[0].max_cpufreq_idx=find_cpu_freq_idx(0,power_mode->cpu_freq_LLmax);
	cur_ppm_limit[0].min_cpufreq_idx=find_cpu_freq_idx(0,power_mode->cpu_freq_LLmin);
	cur_ppm_limit[0].max_cpu_core=power_mode->cpu_core_LL_max;
	cur_ppm_limit[0].min_cpu_core=power_mode->cpu_core_LL_min;
	cur_ppm_limit[1].max_cpufreq_idx=find_cpu_freq_idx(1,power_mode->cpu_freq_Lmax);
	cur_ppm_limit[1].min_cpufreq_idx=find_cpu_freq_idx(1,power_mode->cpu_freq_Lmin);
	cur_ppm_limit[1].max_cpu_core=power_mode->cpu_core_L_max;
	cur_ppm_limit[1].min_cpu_core=power_mode->cpu_core_L_min;

	cur_gpu_freq_idx= find_gpu_freq_idx(power_mode->gpu_freq_lock);
	ppm_user_mode_set_cpu_freq_core(cur_ppm_limit);
	mt_gpufreq_thermal_protect(cur_gpu_freq_idx);
}
ssize_t show_power_mode(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret=0,i=0;
	for (i = 0; i < POWER_MODE_END; i++) {
		ret += sprintf(buf+ret, "%s   ",power_mode_info_tlb[i].mode_name);
	}
	ret += sprintf(buf+ret, "\n");
	return ret;
}

ssize_t store_power_mode(struct kobject *kobj, struct attribute *attr,const char *buf, size_t count)
{
	char str_power_mode[POWER_MODE_LEN];
	int ret , i;
	ret = sscanf(buf, "%8s", str_power_mode);
	if (ret != 1)
		return -EINVAL;

	for (i = 0; i < POWER_MODE_END; i++) {
		if (!strnicmp(power_mode_info_tlb[i].mode_name, str_power_mode, POWER_MODE_LEN)) {
			break;
		}
	}

	if ( i <  POWER_MODE_END) {
        mutex_lock(&power_lock);
		cpu_power_mode = i;
		cur_power_mode = &power_mode_info_tlb[i];
		//power_mode_notifier_call_chain(cpu_power_mode, cur_power_mode);
        //request_power_mode_unlocked(i);
        	set_cur_power_mode(cur_power_mode);
		pr_err("store power_mode to %s \n", cur_power_mode->mode_name);
        mutex_unlock(&power_lock);
	}

	return count;
}
#if 0
static ssize_t show_power_debug(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int ret;

	mutex_lock(&power_lock);
	ret = sprintf(buf, "debug: %u\n", power_mode_debug);
	mutex_unlock(&power_lock);

	return ret;
}

static ssize_t store_power_debug(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int debug;

	mutex_lock(&power_lock);

	ret = sscanf(buf, "%u", &debug);
	if (ret != 1)
		goto fail;

	power_mode_debug = debug;
	pr_info("%s: debug = %u\n", __func__, power_mode_debug);
	mutex_unlock(&power_lock);

	return count;

fail:
	pr_err("usage: echo debug > /sys/power/power_debug\n\n");
	mutex_unlock(&power_lock);

	return -EINVAL;
}

static ssize_t show_power_custom(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int ret;
	struct power_mode_info *p;

	mutex_lock(&power_lock);
	p = &power_mode_info_tlb[POWER_MODE_CUSTOM];
	ret = sprintf(buf, "cpu freq: %u gpu freq: %u cpu num: %u\n",
		p->cpu_freq_little, p->gpu_freq_lock, p->little_num);
	mutex_unlock(&power_lock);

	return ret;
}

static ssize_t store_power_custom(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int little_freq, gpu_freq, little_num;
	struct power_mode_info *p;

	mutex_lock(&power_lock);

	ret = sscanf(buf, "%u %u %u ", &little_freq, &gpu_freq, &little_num);
	if (ret != 3)
		goto fail;

	p = &power_mode_info_tlb[POWER_MODE_CUSTOM];
	p->cpu_freq_little = little_freq;
	p->gpu_freq_lock = gpu_freq;
	p->little_num = little_num;

	if (power_mode_debug)
		pr_info("custom:\t%u\t%u\t%u\n",
			p->cpu_freq_little, p->gpu_freq_lock, p->little_num);

	mutex_unlock(&power_lock);

	return count;

fail:
	pr_err("usage: echo little_freq gpu_freq little_num > /sys/power/power_custom\n\n");
	mutex_unlock(&power_lock);

	return -EINVAL;
}
#endif
define_one_global_rw(power_mode);
//define_one_global_rw(power_debug);
//define_one_global_rw(power_custom);

static struct attribute * g[] = {
	&power_mode.attr,
	//&power_debug.attr,
	//&power_custom.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

int ppm_user_mode_set_cpu_freq_core(struct ppm_cluster_limit *cpu_limit)
{
	int i;
	unsigned int cluster_num = user_mode_policy.req.cluster_num;
	ppm_lock(&user_mode_policy.lock);

		for (i = 0; i < cluster_num; i++) {
			user_mode_policy.req.limit[i].min_cpu_core =     cpu_limit[i].min_cpu_core;
			user_mode_policy.req.limit[i].max_cpu_core =    cpu_limit[i].max_cpu_core;
			user_mode_policy.req.limit[i].min_cpufreq_idx =  cpu_limit[i].min_cpufreq_idx;
			user_mode_policy.req.limit[i].max_cpufreq_idx = cpu_limit[i].max_cpufreq_idx;
		}
	/* unlimited */
	//ut_data.is_core_num_fixed=true;
	/* fire ppm HERE!*/
	user_mode_policy.is_activated = true;
	ppm_unlock(&user_mode_policy.lock);
	ppm_task_wakeup();
}



int power_mode_init(void)
{
	int error;
	
	pr_err("chenhuazhen  Power mode init!");

	error = sysfs_create_group(power_kobj, &attr_group);
	if (error)
		return error;
	
      if (ppm_main_register_policy(&user_mode_policy)) {
		ppm_err("@%s: UT policy register failed\n", __func__);
		error = -EINVAL;
		return error;
	}

	ppm_info("@%s: register %s done!\n", __func__, user_mode_policy.name);
	
	return error;
}

static void __exit power_mode_exit(void)
{
	FUNC_ENTER(FUNC_LV_POLICY);


	ppm_main_unregister_policy(&user_mode_policy);

	FUNC_EXIT(FUNC_LV_POLICY);
}

module_init(power_mode_init);
module_exit(power_mode_exit);

////late_initcall_sync(power_mode_init);
