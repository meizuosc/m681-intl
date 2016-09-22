#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/blkdev.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include "mt_sd.h"
#include "dbg.h"

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include <mach/partition.h>

#define EMMC_SUCCESS 0
#define EMMC_ERROR   -1


struct msdc_host *emmc_get_host(void)
{
	return mtk_msdc_host[0];
}

/* need subtract emmc reserved region for combo feature */
unsigned int hq_emmc_start(char * part)
{
	unsigned int l_addr;
	struct msdc_host *host_ctl;
	struct hd_struct *lp_hd_struct = NULL;


	host_ctl = emmc_get_host();

	BUG_ON(!host_ctl);
	BUG_ON(!host_ctl->mmc);
	BUG_ON(!host_ctl->mmc->card);

	lp_hd_struct = get_part(part);
	if (NULL == lp_hd_struct) {
		pr_debug("not find otp in scatter file error!\n");
		put_part(lp_hd_struct);
		return -EFAULT;
	}

	l_addr = lp_hd_struct->start_sect;        /* block unit */

	if (NULL != lp_hd_struct)
		put_part(lp_hd_struct);

	printk("find emmc start address is 0x%x\n", l_addr);

	return l_addr;
}


unsigned int __emmc_read(char * part,unsigned int blk_offset, unsigned int blks,void *BufferPtr)
{
	unsigned char *l_rcv_buf = (unsigned char *)BufferPtr;
	unsigned int l_addr;
	unsigned int l_otp_size;
	unsigned int l_ret;
	struct scatterlist msdc_sg;
	struct mmc_data msdc_data;
	struct mmc_command msdc_cmd;
	struct mmc_request msdc_mrq;
	struct msdc_host *host_ctl;

	/* check parameter */
	l_addr = hq_emmc_start(part);

	l_addr += blk_offset;

	host_ctl = emmc_get_host();

	BUG_ON(!host_ctl);
	BUG_ON(!host_ctl->mmc);

	printk("[%s:%d]mmc_host is : 0x%p\n", __func__, __LINE__,
		host_ctl->mmc);
	mmc_claim_host(host_ctl->mmc);

	/* make sure access user data area */
	msdc_switch_part(host_ctl, 0);

#if EMMC_OTP_DEBUG
	printk("EMMC_OTP: start MSDC_SINGLE_READ_WRITE!\n");
#endif

	memset(&msdc_data, 0, sizeof(struct mmc_data));
	memset(&msdc_mrq, 0, sizeof(struct mmc_request));
	memset(&msdc_cmd, 0, sizeof(struct mmc_command));

	msdc_mrq.cmd = &msdc_cmd;
	msdc_mrq.data = &msdc_data;

	msdc_data.flags = MMC_DATA_READ;
	if(blks>1)
	{
		msdc_cmd.opcode = MMC_READ_MULTIPLE_BLOCK;
	}
	else
	{
		msdc_cmd.opcode = MMC_READ_SINGLE_BLOCK;
	}
	msdc_data.blocks = blks; 

	memset(l_rcv_buf, 0, blks*512);

	msdc_cmd.arg = l_addr;
	printk("[%s:%d]mmc_host is : 0x%x blk_offset=0x%x,l_otp_size=0x%x\n", __func__, __LINE__,
		l_addr,blk_offset,l_otp_size);

	BUG_ON(!host_ctl->mmc->card);
	if (!mmc_card_blockaddr(host_ctl->mmc->card)) {
		printk("the device is used byte address!\n");
		msdc_cmd.arg <<= 9;
	}

	msdc_cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	msdc_data.stop = NULL;
	msdc_data.blksz = 512;
	msdc_data.sg = &msdc_sg;
	msdc_data.sg_len = 1;

	sg_init_one(&msdc_sg, l_rcv_buf, 512);
	mmc_set_data_timeout(&msdc_data, host_ctl->mmc->card);

	mmc_wait_for_req(host_ctl->mmc, &msdc_mrq);

	mmc_release_host(host_ctl->mmc);

	if (msdc_cmd.error)
		l_ret = msdc_cmd.error;

	if (msdc_data.error)
		l_ret = msdc_data.error;
	else
		l_ret = EMMC_SUCCESS;

	return l_ret;
}

unsigned int __emmc_write(char * part,unsigned int blk_offset, unsigned int blks,void *BufferPtr)
{
	unsigned char *l_send_buf = (unsigned char *)BufferPtr;
	unsigned int l_ret;
	unsigned int l_addr;
	unsigned int l_otp_size;
	struct scatterlist msdc_sg;
	struct mmc_data msdc_data;
	struct mmc_command msdc_cmd;
	struct mmc_request msdc_mrq;
	struct msdc_host *host_ctl;
#ifdef MTK_MSDC_USE_CACHE
	struct mmc_command msdc_sbc;
#endif

	/* check parameter */
	l_addr =  hq_emmc_start(part);

	l_addr += blk_offset;

	host_ctl = emmc_get_host();

	BUG_ON(!host_ctl);
	BUG_ON(!host_ctl->mmc);

	mmc_claim_host(host_ctl->mmc);

	/* make sure access user data area */
	msdc_switch_part(host_ctl, 0);

#if EMMC_OTP_DEBUG
	pr_debug("EMMC_OTP: start MSDC_SINGLE_READ_WRITE!\n");
#endif

	memset(&msdc_data, 0, sizeof(struct mmc_data));
	memset(&msdc_mrq, 0, sizeof(struct mmc_request));
	memset(&msdc_cmd, 0, sizeof(struct mmc_command));
#ifdef MTK_MSDC_USE_CACHE
	memset(&msdc_sbc, 0, sizeof(struct mmc_command));
#endif

	msdc_mrq.cmd = &msdc_cmd;
	msdc_mrq.data = &msdc_data;

	msdc_data.flags = MMC_DATA_WRITE;
#if 0
	if (mmc_card_mmc(host_ctl->mmc->card)
		&& (host_ctl->mmc->card->ext_csd.cache_ctrl & 0x1)) {
		msdc_cmd.opcode = MMC_WRITE_MULTIPLE_BLOCK;
		msdc_mrq.sbc = &msdc_sbc;
		msdc_mrq.sbc->opcode = MMC_SET_BLOCK_COUNT;
		msdc_mrq.sbc->arg = msdc_data.blocks | (1 << 31);
		msdc_mrq.sbc->flags = MMC_RSP_R1 | MMC_CMD_AC;
	} else {
		msdc_cmd.opcode = MMC_WRITE_BLOCK;
	}
#else
	if(blks>1)
	{
		msdc_cmd.opcode = MMC_WRITE_MULTIPLE_BLOCK;
	}
	else
	{
		msdc_cmd.opcode = MMC_WRITE_BLOCK;
	}
#endif
	msdc_data.blocks = 1;

	msdc_cmd.arg = l_addr;

	BUG_ON(!host_ctl->mmc->card);
	if (!mmc_card_blockaddr(host_ctl->mmc->card)) {
		pr_debug("the device is used byte address!\n");
		msdc_cmd.arg <<= 9;
	}

	msdc_cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	msdc_data.stop = NULL;
	msdc_data.blksz = 512;
	msdc_data.sg = &msdc_sg;
	msdc_data.sg_len = 1;

	sg_init_one(&msdc_sg, l_send_buf, 512);
	mmc_set_data_timeout(&msdc_data, host_ctl->mmc->card);

	mmc_wait_for_req(host_ctl->mmc, &msdc_mrq);

	mmc_release_host(host_ctl->mmc);

	if (msdc_cmd.error)
		l_ret = msdc_cmd.error;

	if (msdc_data.error)
		l_ret = msdc_data.error;
	else
		l_ret = EMMC_SUCCESS;

	return l_ret;
}



unsigned int emmc_read(char * part,unsigned int addr, unsigned int len,void *BufferPtr)
{
	int blks=0,blk_offset=0,blk_start_addr=0;
	char * tmp_buf;
	int ret=-1;
	blk_offset=addr/512;
	blk_start_addr=addr%512;
	blks=(len+blk_start_addr)/512+1;
	tmp_buf=vmalloc(blks*512);
	if(tmp_buf==NULL)
	{
		printk("emmc_read  tmp_buf vmalloc fail!\n");
		return EMMC_ERROR;
	}
	memset(tmp_buf,0,blks*512);
	if(EMMC_SUCCESS!=__emmc_read(part,blk_offset,blks,tmp_buf))
	{
		printk("emmc_read fail %s:0x%x,0x%x\n",part,addr,len);
		ret= EMMC_ERROR;
		goto  ERROR;
	}

	memcpy(BufferPtr,tmp_buf+blk_start_addr,len);
	printk("emmc_read success %s:0x%x,0x%x\n",part,addr,len);
	ret=EMMC_SUCCESS;
ERROR:
	vfree(tmp_buf);

	return ret;
}

unsigned int emmc_write(char * part,unsigned int addr, unsigned int len,void *BufferPtr)
{
	int blks=0,blk_offset=0,blk_start_addr=0;
	char * tmp_buf;
	int ret=-1;
	blk_offset=addr/512;
	blk_start_addr=addr%512;
	blks=(len+blk_start_addr)/512+1;

	tmp_buf=vmalloc(blks*512);
	if(tmp_buf==NULL)
	{
		printk("emmc_write  tmp_buf vmalloc fail!\n");
		return EMMC_ERROR;
	}
	memset(tmp_buf,0,blks*512);
	if(blk_start_addr)
	{
		if(EMMC_SUCCESS!=__emmc_read(part,blk_offset,1,tmp_buf))
		{
			printk("emmc_write first blk read error!!\n");
			ret= EMMC_ERROR;
			goto ERROR;
		}
	}
	memcpy(tmp_buf+blk_start_addr,BufferPtr,len);
	if(EMMC_SUCCESS!=__emmc_write(part,blk_offset,blks,tmp_buf))
	{
		printk("emmc_write fail %s:0x%x,0x%x\n",part,addr,len);
		ret= EMMC_ERROR;
		goto ERROR;
	}
	printk("emmc_read success %s:0x%x,0x%x\n",part,addr,len);
	ret=EMMC_SUCCESS;
ERROR:
	vfree(tmp_buf);

	return ret;
}




static int hq_emmc_probe(struct platform_device *pdev)
{
	pr_debug("in hq_emmc_probe function\n");

	return 0;
}

static int hq_emmc_remove(struct platform_device *pdev)
{
	return 0;
}


static int __init hq_emmc_init(void)
{
	int err = 0;

	pr_debug(" hq_emmc_init init\n");


	return 0;
}

static void __exit hq_emmc_exit(void)
{
	pr_debug("hq_emmc_exit exit\n");

}


module_init(hq_emmc_init);
module_exit(hq_emmc_exit);
