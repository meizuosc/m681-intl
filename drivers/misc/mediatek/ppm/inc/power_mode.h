#ifndef POWER_MODE
#define POWER_MODE
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm_qos.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/cpufreq.h>
#include <linux/notifier.h>
#endif

#define POWER_MODE_LEN	(8)

#if 0
struct power_mode_info {
	char		*mode_name;
	unsigned int	cpu_freq_big;
	unsigned int	cpu_freq_little;
	unsigned int	gpu_freq_lock;
	unsigned int	little_num;
	unsigned int	big_num;
	unsigned int	little_turbo;
	unsigned int	big_turbo;
	unsigned int	turbo_temp;
};

#else
 
struct power_mode_info {
	char		*mode_name;
	unsigned int	cpu_freq_LLmax;
	unsigned int	cpu_freq_LLmin;
	unsigned int	cpu_freq_Lmax;
	unsigned int	cpu_freq_Lmin;
	
	unsigned int    cpu_core_LL_max;
	unsigned int    cpu_core_LL_min;
	unsigned int    cpu_core_L_max;
	unsigned int    cpu_core_L_min;
	unsigned int	gpu_freq_lock;
};
#endif

enum power_mode_idx {
	POWER_MODE_0,
	POWER_MODE_1,
	POWER_MODE_2,
	//POWER_MODE_CUSTOM,
	POWER_MODE_END,
};

int power_mode_init(void);
void show_power_mode_list(void);

extern void mt_gpufreq_thermal_protect(unsigned int limited_power);
extern int ppm_user_mode_set_cpu_freq_core(struct ppm_cluster_limit *cpu_limit);



