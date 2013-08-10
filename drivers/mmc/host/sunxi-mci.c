/*
 * drivers/mmc/host/sunxi-mci.c
 * (C) Copyright 2007-2011
 * Reuuimlla Technology Co., Ltd. <www.reuuimllatech.com>
 * Aaron.Maoye <leafy.myeh@reuuimllatech.com>
 *
 * description for this code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>

#include <linux/mmc/host.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>

#include <asm/cacheflush.h>
#include <asm/uaccess.h>

#include "sunxi-mci.h"

struct sunxi_mmc {
	struct clk		*clk_ahb;
	struct clk		*clk_mod;
	void __iomem		*membase;
	struct regulator	*regulator;

	int			cd_pin;
	int			wp_pin;
	bool			cd_active_high;

	bool			present;

	dma_addr_t	sg_dma;
	void		*sg_cpu;
	struct platform_device	*pdev;
};

static s32 sw_mci_init_host(struct sunxi_mmc_host* smc_host)
{
	u32 rval;

	SMC_DBG(smc_host, "MMC Driver init host %d\n", smc_host->pdev->id);

	/* reset controller */
	rval = mci_readl(smc_host, REG_GCTRL) | SDXC_HWReset;
	mci_writel(smc_host, REG_GCTRL, rval);

	mci_writel(smc_host, REG_FTRGL, 0x70008);
	mci_writel(smc_host, REG_TMOUT, 0xffffffff);
	mci_writel(smc_host, REG_IMASK, 0);
	mci_writel(smc_host, REG_RINTR, 0xffffffff);
	mci_writel(smc_host, REG_DBGC, 0xdeb);
	mci_writel(smc_host, REG_FUNS, 0xceaa0000);
	rval = mci_readl(smc_host, REG_GCTRL)|SDXC_INTEnb;
	rval &= ~SDXC_AccessDoneDirect;
	mci_writel(smc_host, REG_GCTRL, rval);

	smc_host->voltage = SDC_WOLTAGE_OFF;
//	if (smc_host->pdata->isiodev)
//		smc_host->io_flag = 1;
	return 0;
}

s32 sw_mci_exit_host(struct sunxi_mmc_host* smc_host)
{
	u32 rval;

	SMC_DBG(smc_host, "MMC Driver exit host %d\n", smc_host->pdev->id);
	smc_host->ferror = 0;
	smc_host->voltage = SDC_WOLTAGE_OFF;

	rval = mci_readl(smc_host, REG_GCTRL) | SDXC_HWReset;
	mci_writel(smc_host, REG_GCTRL, SDXC_HWReset);
	return 0;
}

s32 sw_mci_set_vddio(struct sunxi_mmc_host* smc_host, u32 vdd)
{
	char* vddstr[] = {"3.3V", "1.8V", "1.2V", "OFF"};
	static u32 on[4] = {0};
	u32 id = smc_host->pdev->id;

	if (smc_host->regulator == NULL)
		return 0;
	BUG_ON(vdd > SDC_WOLTAGE_OFF);
	switch (vdd) {
		case SDC_WOLTAGE_3V3:
			regulator_set_voltage(smc_host->regulator, 3300000, 3300000);
			if (!on[id]) {
				SMC_DBG(smc_host, "regulator on\n");
				regulator_enable(smc_host->regulator);
				on[id] = 1;
			}
			break;
		case SDC_WOLTAGE_1V8:
			regulator_set_voltage(smc_host->regulator, 1800000, 1800000);
			if (!on[id]) {
				SMC_DBG(smc_host, "regulator on\n");
				regulator_enable(smc_host->regulator);
				on[id] = 1;
			}
			break;
		case SDC_WOLTAGE_1V2:
			regulator_set_voltage(smc_host->regulator, 1200000, 1200000);
			if (!on[id]) {
				SMC_DBG(smc_host, "regulator on\n");
				regulator_enable(smc_host->regulator);
				on[id] = 1;
			}
			break;
		case SDC_WOLTAGE_OFF:
			if (on[id]) {
				SMC_DBG(smc_host, "regulator off\n");
				regulator_force_disable(smc_host->regulator);
				on[id] = 0;
			}
			break;
	}
	SMC_MSG(smc_host, "sdc%d switch io voltage to %s\n", smc_host->pdev->id, vddstr[vdd]);
	return 0;
}

s32 sw_mci_update_clk(struct sunxi_mmc_host* smc_host)
{
  	u32 rval;
  	s32 expire = jiffies + msecs_to_jiffies(1000);	//1000ms timeout
  	s32 ret = 0;

  	rval = SDXC_Start|SDXC_UPCLKOnly|SDXC_WaitPreOver;
	if (smc_host->voltage_switching)
		rval |= SDXC_VolSwitch;
	mci_writel(smc_host, REG_CMDR, rval);

	do {
		rval = mci_readl(smc_host, REG_CMDR);
	} while (jiffies < expire && (rval & SDXC_Start));

	if (rval & SDXC_Start) {
		smc_host->ferror = 1;
		SMC_ERR(smc_host, "update clock timeout, fatal error!!!\n");
		ret = -1;
	}

	return ret;
}

/* /\* UHS-I Operation Modes */
/*  * DS		25MHz	12.5MB/s	3.3V */
/*  * HS		50MHz	25MB/s		3.3V */
/*  * SDR12	25MHz	12.5MB/s	1.8V */
/*  * SDR25	50MHz	25MB/s		1.8V */
/*  * SDR50	100MHz	50MB/s		1.8V */
/*  * SDR104	208MHz	104MB/s		1.8V */
/*  * DDR50	50MHz	50MB/s		1.8V */
/*  * MMC Operation Modes */
/*  * DS		26MHz	26MB/s		3/1.8/1.2V */
/*  * HS		52MHz	52MB/s		3/1.8/1.2V */
/*  * HSDDR	52MHz	104MB/s		3/1.8/1.2V */
/*  * HS200	200MHz	200MB/s		1.8/1.2V */
/*  * */
/*  * Spec. Timing */
/*  * SD3.0 */
/*  * Fcclk    Tcclk   Fsclk   Tsclk   Tis     Tih     odly  RTis     RTih */
/*  * 400K     2.5us   24M     41ns    5ns     5ns     1     2209ns   41ns */
/*  * 25M      40ns    600M    1.67ns  5ns     5ns     3     14.99ns  5.01ns */
/*  * 50M      20ns    600M    1.67ns  6ns     2ns     3     14.99ns  5.01ns */
/*  * 50MDDR   20ns    600M    1.67ns  6ns     0.8ns   2     6.67ns   3.33ns */
/*  * 104M     9.6ns   600M    1.67ns  3ns     0.8ns   1     7.93ns   1.67ns */
/*  * 208M     4.8ns   600M    1.67ns  1.4ns   0.8ns   1     3.33ns   1.67ns */

/*  * 25M      40ns    300M    3.33ns  5ns     5ns     2     13.34ns   6.66ns */
/*  * 50M      20ns    300M    3.33ns  6ns     2ns     2     13.34ns   6.66ns */
/*  * 50MDDR   20ns    300M    3.33ns  6ns     0.8ns   1     6.67ns    3.33ns */
/*  * 104M     9.6ns   300M    3.33ns  3ns     0.8ns   0     7.93ns    1.67ns */
/*  * 208M     4.8ns   300M    3.33ns  1.4ns   0.8ns   0     3.13ns    1.67ns */

/*  * eMMC4.5 */
/*  * 400K     2.5us   24M     41ns    3ns     3ns     1     2209ns    41ns */
/*  * 25M      40ns    600M    1.67ns  3ns     3ns     3     14.99ns   5.01ns */
/*  * 50M      20ns    600M    1.67ns  3ns     3ns     3     14.99ns   5.01ns */
/*  * 50MDDR   20ns    600M    1.67ns  2.5ns   2.5ns   2     6.67ns    3.33ns */
/*  * 200M     5ns     600M    1.67ns  1.4ns   0.8ns   1     3.33ns    1.67ns */
/*  *\/ */
/* struct sw_mmc_clk_dly { */
/* 	u32 mode; */
/* #define MMC_CLK_400K		0 */
/* #define MMC_CLK_25M		1 */
/* #define MMC_CLK_50M		2 */
/* #define MMC_CLK_50MDDR		3 */
/* #define MMC_CLK_50MDDR_8BIT	4 */
/* #define MMC_CLK_100M		5 */
/* #define MMC_CLK_200M		6 */
/* #define MMC_CLK_MOD_NUM		7 */
/* 	u32 oclk_dly; */
/* 	u32 sclk_dly; */
/* } mmc_clk_dly [MMC_CLK_MOD_NUM] = { */
/* 	{MMC_CLK_400K,        0, 7}, */
/* 	{MMC_CLK_25M,         0, 5}, */
/* 	{MMC_CLK_50M,         3, 5}, */
/* 	{MMC_CLK_50MDDR,      2, 4}, */
/* 	{MMC_CLK_50MDDR_8BIT, 2, 4}, */
/* 	{MMC_CLK_100M,        1, 4}, */
/* 	{MMC_CLK_200M,        1, 4}, */
/* }; */

/* s32 sw_mci_set_clk_dly(struct sunxi_mmc_host* smc_host, u32 oclk_dly, u32 sclk_dly) */
/* { */
/* 	u32 smc_no = smc_host->pdev->id; */
/* 	void __iomem *mclk_base = __io_address(0x01c20088 + 0x4 * smc_no); */
/* 	u32 rval; */
/* 	unsigned long iflags; */

/* 	spin_lock_irqsave(&smc_host->lock, iflags); */
/* 	rval = readl(mclk_base); */
/* 	rval &= ~((0x7U << 8) | (0x7U << 20)); */
/* 	rval |= (oclk_dly << 8) | (sclk_dly << 20); */
/* 	writel(rval, mclk_base); */
/* 	spin_unlock_irqrestore(&smc_host->lock, iflags); */

/* 	smc_host->oclk_dly = oclk_dly; */
/* 	smc_host->sclk_dly = sclk_dly; */
/* 	SMC_DBG(smc_host, "oclk_dly %d, sclk_dly %d\n", oclk_dly, sclk_dly); */
/* 	return 0; */
/* } */

/* s32 sw_mci_oclk_onoff(struct sunxi_mmc_host* smc_host, u32 oclk_en, u32 pwr_save) */
/* { */
/* 	u32 rval = mci_readl(smc_host, REG_CLKCR); */
/* 	rval &= ~(SDXC_CardClkOn | SDXC_LowPowerOn); */
/* 	if (oclk_en) */
/* 		rval |= SDXC_CardClkOn; */
/* 	if (pwr_save || !smc_host->io_flag) */
/* 		rval |= SDXC_LowPowerOn; */
/* 	mci_writel(smc_host, REG_CLKCR, rval); */
/* 	sw_mci_update_clk(smc_host); */
/* 	return 0; */
/* } */

static void sw_mci_send_cmd(struct sunxi_mmc_host* smc_host, struct mmc_command* cmd)
{
	u32 imask = SDXC_IntErrBit;
	u32 cmd_val = SDXC_Start|(cmd->opcode&0x3f);
	unsigned long iflags;
	u32 wait = SDC_WAIT_NONE;

	wait = SDC_WAIT_CMD_DONE;
	if (cmd->opcode == MMC_GO_IDLE_STATE) {
		cmd_val |= SDXC_SendInitSeq;
		imask |= SDXC_CmdDone;
	}

	if (cmd->opcode == SD_SWITCH_VOLTAGE) {
		cmd_val |= SDXC_VolSwitch;
		imask |= SDXC_VolChgDone;
		smc_host->voltage_switching = 1;
		wait = SDC_WAIT_SWITCH1V8;
		/* switch controller to high power mode */
//		sw_mci_oclk_onoff(smc_host, 1, 0);
	}

	if (cmd->flags & MMC_RSP_PRESENT) {
		cmd_val |= SDXC_RspExp;
		if (cmd->flags & MMC_RSP_136)
			cmd_val |= SDXC_LongRsp;
		if (cmd->flags & MMC_RSP_CRC)
			cmd_val |= SDXC_CheckRspCRC;

		if ((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC) {
			cmd_val |= SDXC_DataExp | SDXC_WaitPreOver;
			wait = SDC_WAIT_DATA_OVER;
			if (cmd->data->flags & MMC_DATA_STREAM) {
				imask |= SDXC_AutoCMDDone;
				cmd_val |= SDXC_Seqmod | SDXC_SendAutoStop;
				wait = SDC_WAIT_AUTOCMD_DONE;
			}
			if (cmd->data->stop) {
				imask |= SDXC_AutoCMDDone;
				cmd_val |= SDXC_SendAutoStop;
				wait = SDC_WAIT_AUTOCMD_DONE;
			} else
				imask |= SDXC_DataOver;

			if (cmd->data->flags & MMC_DATA_WRITE)
				cmd_val |= SDXC_Write;
			else
				wait |= SDC_WAIT_DMA_DONE;
		} else
			imask |= SDXC_CmdDone;

	} else
		imask |= SDXC_CmdDone;
	SMC_DBG(smc_host, "smc %d cmd %d(%08x) arg %x ie 0x%08x wt %x len %d\n",
		smc_host->pdev->id, cmd_val&0x3f, cmd->arg, cmd_val, imask, wait,
		smc_host->mrq->data ? smc_host->mrq->data->blksz * smc_host->mrq->data->blocks : 0);
	spin_lock_irqsave(&smc_host->lock, iflags);
	#ifdef CONFIG_CACULATE_TRANS_TIME
	begin_time = over_time = end_time = cpu_clock(smp_processor_id());
	#endif
	smc_host->wait = wait;
	smc_host->state = SDC_STATE_SENDCMD;
	mci_writew(smc_host, REG_IMASK, imask);
	mci_writel(smc_host, REG_CARG, cmd->arg);
	mci_writel(smc_host, REG_CMDR, cmd_val);
	smp_wmb();
	spin_unlock_irqrestore(&smc_host->lock, iflags);
}

static void sw_mci_init_idma_des(struct sunxi_mmc_host* smc_host, struct mmc_data* data)
{
	struct sunxi_mmc_idma_des* pdes = (struct sunxi_mmc_idma_des*)smc_host->sg_cpu;
	struct sunxi_mmc_idma_des* pdes_pa = (struct sunxi_mmc_idma_des*)smc_host->sg_dma;
	u32 des_idx = 0;
	u32 buff_frag_num = 0;
	u32 remain;
	u32 i, j;
	u32 config;

	for (i=0; i<data->sg_len; i++) {
		buff_frag_num = data->sg[i].length >> SDXC_DES_NUM_SHIFT;
		remain = data->sg[i].length & (SDXC_DES_BUFFER_MAX_LEN-1);
		if (remain)
			buff_frag_num ++;
		else
			remain = SDXC_DES_BUFFER_MAX_LEN;

		for (j=0; j < buff_frag_num; j++, des_idx++) {
			memset((void*)&pdes[des_idx], 0, sizeof(struct sunxi_mmc_idma_des));
			config = SDXC_IDMAC_DES0_CH|SDXC_IDMAC_DES0_OWN|SDXC_IDMAC_DES0_DIC;

		    	if (buff_frag_num > 1 && j != buff_frag_num-1)
				pdes[des_idx].data_buf1_sz = SDXC_DES_BUFFER_MAX_LEN;
		    	else
				pdes[des_idx].data_buf1_sz = remain;

			pdes[des_idx].buf_addr_ptr1 = sg_dma_address(&data->sg[i])
							+ j * SDXC_DES_BUFFER_MAX_LEN;
			if (i==0 && j==0)
				config |= SDXC_IDMAC_DES0_FD;

			if ((i == data->sg_len-1) && (j == buff_frag_num-1)) {
				config &= ~SDXC_IDMAC_DES0_DIC;
				config |= SDXC_IDMAC_DES0_LD|SDXC_IDMAC_DES0_ER;
				pdes[des_idx].buf_addr_ptr2 = 0;
				#ifdef CONFIG_CACULATE_TRANS_TIME
				last_pdes = &pdes[des_idx];
				#endif
			} else {
				pdes[des_idx].buf_addr_ptr2 = (u32)&pdes_pa[des_idx+1];
			}
			pdes[des_idx].config = config;
			SMC_INFO(smc_host, "sg %d, frag %d, remain %d, des[%d](%08x): "
		    		"[0] = %08x, [1] = %08x, [2] = %08x, [3] = %08x\n", i, j, remain,
				des_idx, (u32)&pdes[des_idx],
				(u32)((u32*)&pdes[des_idx])[0], (u32)((u32*)&pdes[des_idx])[1],
				(u32)((u32*)&pdes[des_idx])[2], (u32)((u32*)&pdes[des_idx])[3]);
		}
	}
	smp_wmb();
	return;
}

static int sw_mci_prepare_dma(struct sunxi_mmc_host* smc_host, struct mmc_data* data)
{
	u32 dma_len;
	u32 i;
	u32 temp;
	struct scatterlist *sg;

	if (smc_host->sg_cpu == NULL)
		return -ENOMEM;

	dma_len = dma_map_sg(mmc_dev(smc_host->mmc), data->sg, data->sg_len,
			(data->flags & MMC_DATA_WRITE) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	if (dma_len == 0) {
		SMC_ERR(smc_host, "no dma map memory\n");
		return -ENOMEM;
	}

	for_each_sg(data->sg, sg, data->sg_len, i) {
		if (sg->offset & 3 || sg->length & 3) {
			SMC_ERR(smc_host, "unaligned scatterlist: os %x length %d\n",
				sg->offset, sg->length);
			return -EINVAL;
		}
	}

	sw_mci_init_idma_des(smc_host, data);
	temp = mci_readl(smc_host, REG_GCTRL);
	temp |= SDXC_DMAEnb;
	mci_writel(smc_host, REG_GCTRL, temp);
	temp |= SDXC_DMAReset;
	mci_writel(smc_host, REG_GCTRL, temp);
	mci_writel(smc_host, REG_DMAC, SDXC_IDMACSoftRST);
	temp = SDXC_IDMACFixBurst|SDXC_IDMACIDMAOn;
	mci_writel(smc_host, REG_DMAC, temp);
	temp = mci_readl(smc_host, REG_IDIE);
	temp &= ~(SDXC_IDMACReceiveInt|SDXC_IDMACTransmitInt);
	if (data->flags & MMC_DATA_WRITE)
		temp |= SDXC_IDMACTransmitInt;
	else
		temp |= SDXC_IDMACReceiveInt;
	mci_writel(smc_host, REG_IDIE, temp);

	//write descriptor address to register
	mci_writel(smc_host, REG_DLBA, smc_host->sg_dma);
//	mci_writel(smc_host, REG_FTRGL, smc_host->pdata->dma_tl);

	return 0;
}

int sw_mci_send_manual_stop(struct sunxi_mmc_host* smc_host, struct mmc_request* req)
{
	struct mmc_data* data = req->data;
	u32 cmd_val = SDXC_Start | SDXC_RspExp | SDXC_StopAbortCMD
			| SDXC_CheckRspCRC | MMC_STOP_TRANSMISSION;
	u32 iflags = 0;
	u32 imask = 0;
	int ret = 0;
	u32 expire = jiffies + msecs_to_jiffies(1000);

	if (!data) {
		SMC_ERR(smc_host, "no data request\n");
		return -1;
	}
	/* disable interrupt */
	imask = mci_readw(smc_host, REG_IMASK);
	mci_writew(smc_host, REG_IMASK, 0);

	mci_writel(smc_host, REG_CARG, 0);
	mci_writel(smc_host, REG_CMDR, cmd_val);
	do {
		iflags = mci_readw(smc_host, REG_RINTR);
	} while(!(iflags & (SDXC_CmdDone | SDXC_IntErrBit)) && jiffies < expire);

	if (iflags & SDXC_IntErrBit) {
		SMC_ERR(smc_host, "sdc %d send stop command failed\n", smc_host->pdev->id);
		ret = -1;
	}

	if (req->stop)
		req->stop->resp[0] = mci_readl(smc_host, REG_RESP0);

	mci_writew(smc_host, REG_RINTR, iflags);

	/* enable interrupt */
	mci_writew(smc_host, REG_IMASK, imask);

	return ret;
}

void sw_mci_dump_errinfo(struct sunxi_mmc_host* smc_host)
{
	SMC_ERR(smc_host, "smc %d err, cmd %d, %s%s%s%s%s%s%s%s%s%s !!\n",
		smc_host->pdev->id, smc_host->mrq->cmd ? smc_host->mrq->cmd->opcode : -1,
		smc_host->int_sum & SDXC_RespErr     ? " RE"     : "",
		smc_host->int_sum & SDXC_RespCRCErr  ? " RCE"    : "",
		smc_host->int_sum & SDXC_DataCRCErr  ? " DCE"    : "",
		smc_host->int_sum & SDXC_RespTimeout ? " RTO"    : "",
		smc_host->int_sum & SDXC_DataTimeout ? " DTO"    : "",
		smc_host->int_sum & SDXC_DataStarve  ? " DS"     : "",
		smc_host->int_sum & SDXC_FIFORunErr  ? " FE"     : "",
		smc_host->int_sum & SDXC_HardWLocked ? " HL"     : "",
		smc_host->int_sum & SDXC_StartBitErr ? " SBE"    : "",
		smc_host->int_sum & SDXC_EndBitErr   ? " EBE"    : ""
		);
}

s32 sw_mci_wait_access_done(struct sunxi_mmc_host* smc_host)
{
	s32 own_set = 0;
	unsigned long expire = jiffies + msecs_to_jiffies(5);
	while (!(mci_readl(smc_host, REG_GCTRL) & SDXC_MemAccessDone) && jiffies < expire);
	if (!(mci_readl(smc_host, REG_GCTRL) & SDXC_MemAccessDone)) {
		SMC_MSG(smc_host, "wait memory access done timeout !!\n");
	}
	#ifdef CONFIG_CACULATE_TRANS_TIME
	expire = jiffies + msecs_to_jiffies(5);
	while (last_pdes->config & SDXC_IDMAC_DES0_OWN && jiffies < expire) {
		own_set = 1;
	}
	if (jiffies > expire) {
		SMC_MSG(smc_host, "wait last pdes own bit timeout, lc %x, rc %x\n",
			last_pdes->config, mci_readl(smc_host, REG_CHDA));
	}

	end_time = cpu_clock(smp_processor_id());
	if (own_set || (end_time - over_time) > 1000000) {
		SMC_MSG(smc_host, "Own set %x, t1 %llu t2 %llu t3 %llu, d1 %llu d2 %llu, bcnt %x\n",
			own_set, begin_time, over_time, end_time, over_time - begin_time,
			end_time - over_time, mci_readl(smc_host, REG_BCNTR));
		own_set = 0;
	}
	#endif
	return own_set;
}

s32 sw_mci_request_done(struct sunxi_mmc_host* smc_host)
{
	struct mmc_request* req = smc_host->mrq;
	u32 temp;
	s32 ret = 0;

	if (smc_host->int_sum & SDXC_IntErrBit) {
		/* if we got response timeout error information, we should check
		   if the command done status has been set. if there is no command
		   done information, we should wait this bit to be set */
		if ((smc_host->int_sum & SDXC_RespTimeout) && !(smc_host->int_sum & SDXC_CmdDone)) {
			u32 rint;
			u32 expire = jiffies + 1;
			do {
				rint = mci_readl(smc_host, REG_RINTR);
			} while (jiffies < expire && !(rint & SDXC_CmdDone));
		}

		sw_mci_dump_errinfo(smc_host);
		if (req->data)
			SMC_ERR(smc_host, "In data %s operation\n",
				req->data->flags & MMC_DATA_WRITE ? "write" : "read");
		ret = -1;
		goto out;
	}

	if (req->cmd) {
		if (req->cmd->flags & MMC_RSP_136) {
			req->cmd->resp[0] = mci_readl(smc_host, REG_RESP3);
			req->cmd->resp[1] = mci_readl(smc_host, REG_RESP2);
			req->cmd->resp[2] = mci_readl(smc_host, REG_RESP1);
			req->cmd->resp[3] = mci_readl(smc_host, REG_RESP0);
		} else {
			req->cmd->resp[0] = mci_readl(smc_host, REG_RESP0);
		}
	}

out:
	if (req->data) {
		struct mmc_data* data = req->data;

		sw_mci_wait_access_done(smc_host);
		mci_writel(smc_host, REG_IDST, 0x337);
		mci_writel(smc_host, REG_IDIE, 0);
		mci_writel(smc_host, REG_DMAC, 0);
		temp = mci_readl(smc_host, REG_GCTRL);
		mci_writel(smc_host, REG_GCTRL, temp|SDXC_DMAReset);
		temp &= ~SDXC_DMAEnb;
		mci_writel(smc_host, REG_GCTRL, temp);
		temp |= SDXC_FIFOReset;
		mci_writel(smc_host, REG_GCTRL, temp);
		dma_unmap_sg(mmc_dev(smc_host->mmc), data->sg, data->sg_len,
                                data->flags & MMC_DATA_WRITE ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	}

	mci_writew(smc_host, REG_IMASK, 0);
	if (smc_host->int_sum & (SDXC_RespErr | SDXC_HardWLocked | SDXC_RespTimeout)) {
		SMC_DBG(smc_host, "sdc %d abnormal status: %s\n", smc_host->pdev->id,
			smc_host->int_sum & SDXC_HardWLocked ? "HardWLocked" : "RespErr");
	}

	mci_writew(smc_host, REG_RINTR, 0xffff);

	SMC_DBG(smc_host, "smc %d done, resp %08x %08x %08x %08x\n", smc_host->pdev->id,
		req->cmd->resp[0], req->cmd->resp[1], req->cmd->resp[2], req->cmd->resp[3]);

	if (req->data  && (smc_host->int_sum & SDXC_IntErrBit)) {
		SMC_MSG(smc_host, "found data error, need to send stop command !!\n");
		sw_mci_send_manual_stop(smc_host, req);
	}

	return ret;
}

static int sw_mci_resource_request(struct sunxi_mmc *host)
{
	struct platform_device *pdev = host->pdev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->membase = devm_request_and_ioremap(&pdev->dev, res);
	if (!host->membase) {
		dev_err(&pdev->dev, "Failed to map memory\n");
		return -ENOMEM;
	}

	host->clk_ahb = of_clk_get(np, 0);
	if (IS_ERR(host->clk_ahb)) {
		dev_err(&pdev->dev, "Couldn't get AHB clock\n");
		return PTR_ERR(host->clk_ahb);
	}

	host->clk_mod = of_clk_get(np, 1);
	if (IS_ERR(host->clk_mod)) {
		dev_err(&pdev->dev, "Couldn't get module clock\n");
		ret = PTR_ERR(host->clk_mod);
		goto free_ahb_clk;
	}

	host->sg_cpu = dma_alloc_writecombine(NULL, PAGE_SIZE,
					      &host->sg_dma, GFP_KERNEL);
	if (!host->sg_cpu) {
		dev_err(&pdev->dev, "Failed to allocate DMA descriptor\n");
		goto free_mod_clk;
	}

	host->regulator = devm_regulator_get(&pdev->dev, "mmc");
	if (IS_ERR(host->regulator)) {
		if (PTR_ERR(host->regulator) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		else
			dev_info(&pdev->dev, "no regulator found\n");
	}

	return 0;

free_mod_clk:
	clk_put(host->clk_mod);
free_ahb_clk:
	clk_put(host->clk_ahb);
	return ret;
}

static int sw_mci_resource_release(struct sunxi_mmc *host)
{
	if (host->sg_cpu) {
		dma_free_coherent(NULL, PAGE_SIZE,
				  host->sg_cpu, host->sg_dma);
	}

	clk_put(host->clk_mod);
	clk_put(host->clk_ahb);

	return 0;
}

static void sw_mci_finalize_request(struct sunxi_mmc_host *smc_host)
{
	struct mmc_request* mrq = smc_host->mrq;
	unsigned long iflags;

	spin_lock_irqsave(&smc_host->lock, iflags);
	if (smc_host->wait != SDC_WAIT_FINALIZE) {
		spin_unlock_irqrestore(&smc_host->lock, iflags);
		SMC_MSG(smc_host, "nothing finalize, wt %x, st %d\n",
				smc_host->wait, smc_host->state);
		return;
	}
	smc_host->wait = SDC_WAIT_NONE;
	smc_host->state = SDC_STATE_IDLE;
	smc_host->trans_done = 0;
	smc_host->dma_done = 0;
	spin_unlock_irqrestore(&smc_host->lock, iflags);

	sw_mci_request_done(smc_host);
	if (smc_host->error) {
		mrq->cmd->error = -ETIMEDOUT;
		if (mrq->data)
			mrq->data->error = -ETIMEDOUT;
		if (mrq->stop)
			mrq->stop->error = -ETIMEDOUT;
	} else {
		if (mrq->data)
			mrq->data->bytes_xfered = (mrq->data->blocks * mrq->data->blksz);
	}

	smc_host->mrq = NULL;
	smc_host->error = 0;
	smc_host->int_sum = 0;
	smp_wmb();
	mmc_request_done(smc_host->mmc, mrq);
	return;
}

static s32 sw_mci_get_ro(struct mmc_host *mmc)
{
	struct sunxi_mmc *host = mmc_priv(mmc);
	int read_only = -ENODEV;

	if (gpio_is_valid(host->wp_pin)) {
		read_only = gpio_get_value(host->wp_pin);
		dev_dbg(&host->pdev->dev, "card is %s\n",
			read_only ? "read-only" : "read-write");
	}

	return read_only;
}

static int sw_mci_get_cd(struct mmc_host *mmc)
{
	struct sunxi_mmc *host = mmc_priv(mmc);
	int present = -ENODEV;

	if (gpio_is_valid(host->cd_pin)) {
		present = !(gpio_get_value(host->cd_pin) ^
			    host->cd_active_high);
		dev_dbg(&mmc->class_dev, "card is %spresent\n",
				present ? "" : "not ");
	}

	return present;
}

static void sw_mci_detect_change(unsigned long data)
{
	struct mmc_host *mmc = (struct mmc_host *)data;
	struct sunxi_mmc *host = mmc_priv(mmc);
	bool present;

	present = sw_mci_get_cd(mmc);

	dev_vdbg(&host->pdev->dev, "detect change: %d (was %d)\n",
		 present, host->present);

	if (host->present ^ present) {
		dev_dbg(&host->pdev->dev, "card %s\n",
			present ? "inserted" : "removed");

		host->present = present;

		if (present)
			mmc_detect_change(mmc, HZ * 10);
		else
			mmc_detect_change(mmc, HZ);
	}
}

static irqreturn_t sw_mci_irq(int irq, void *dev_id)
{
	struct sunxi_mmc_host *smc_host = dev_id;
	u32 sdio_int = 0;
	u32 raw_int;
	u32 msk_int;
	u32 idma_inte;
	u32 idma_int;

	spin_lock(&smc_host->lock);

	idma_int  = mci_readl(smc_host, REG_IDST);
	idma_inte = mci_readl(smc_host, REG_IDIE);
	raw_int   = mci_readl(smc_host, REG_RINTR);
	msk_int   = mci_readl(smc_host, REG_MISTA);
	if (!msk_int && !idma_int) {
		SMC_MSG(smc_host, "sdc%d nop irq: ri %08x mi %08x ie %08x idi %08x\n",
			smc_host->pdev->id, raw_int, msk_int, idma_inte, idma_int);
		spin_unlock(&smc_host->lock);
		return IRQ_HANDLED;
	}

	smc_host->int_sum |= raw_int;
	SMC_INFO(smc_host, "smc %d irq, ri %08x(%08x) mi %08x ie %08x idi %08x\n",
		smc_host->pdev->id, raw_int, smc_host->int_sum,
		msk_int, idma_inte, idma_int);

	if (msk_int & SDXC_SDIOInt) {
		sdio_int = 1;
		mci_writel(smc_host, REG_RINTR, SDXC_SDIOInt);
		goto sdio_out;
	}

	if (smc_host->wait == SDC_WAIT_NONE && !sdio_int) {
		SMC_ERR(smc_host, "smc %x, nothing to complete, ri %08x, "
			"mi %08x\n", smc_host->pdev->id, raw_int, msk_int);
		goto irq_out;
	}

	if ((raw_int & SDXC_IntErrBit) || (idma_int & SDXC_IDMA_ERR)) {
		smc_host->error = raw_int & SDXC_IntErrBit;
		smc_host->wait = SDC_WAIT_FINALIZE;
		smc_host->state = SDC_STATE_CMDDONE;
		goto irq_out;
	}
	if (idma_int & (SDXC_IDMACTransmitInt|SDXC_IDMACReceiveInt))
		smc_host->dma_done = 1;
	if (msk_int & (SDXC_AutoCMDDone|SDXC_DataOver|SDXC_CmdDone|SDXC_VolChgDone))
		smc_host->trans_done = 1;
	if ((smc_host->trans_done && (smc_host->wait == SDC_WAIT_AUTOCMD_DONE
					|| smc_host->wait == SDC_WAIT_DATA_OVER
					|| smc_host->wait == SDC_WAIT_CMD_DONE
					|| smc_host->wait == SDC_WAIT_SWITCH1V8))
		|| (smc_host->trans_done && smc_host->dma_done && (smc_host->wait & SDC_WAIT_DMA_DONE))) {
		smc_host->wait = SDC_WAIT_FINALIZE;
		smc_host->state = SDC_STATE_CMDDONE;

		#ifdef CONFIG_CACULATE_TRANS_TIME
		over_time = cpu_clock(smp_processor_id());
		#endif
	}

irq_out:
	mci_writel(smc_host, REG_RINTR, msk_int&(~SDXC_SDIOInt));
	mci_writel(smc_host, REG_IDST, idma_int);

	if (smc_host->wait == SDC_WAIT_FINALIZE) {
		smp_wmb();
		mci_writew(smc_host, REG_IMASK, 0);
		tasklet_schedule(&smc_host->tasklet);
	}

sdio_out:
	spin_unlock(&smc_host->lock);

	if (sdio_int)
		mmc_signal_sdio_irq(smc_host->mmc);

	return IRQ_HANDLED;
}

static void sw_mci_tasklet(unsigned long data)
{
	struct sunxi_mmc_host *smc_host = (struct sunxi_mmc_host *) data;
	sw_mci_finalize_request(smc_host);
}

static void sw_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sunxi_mmc *host = mmc_priv(mmc);
	char* bus_mode[] = {"", "OD", "PP"};
	char* pwr_mode[] = {"OFF", "UP", "ON"};
	char* vdd[] = {"3.3V", "1.8V", "1.2V"};
	char* timing[] = {"LEGACY(SDR12)", "MMC-HS(SDR20)", "SD-HS(SDR25)",
			"UHS-SDR50", "UHS-SDR104", "UHS-DDR50", "MMC-HS200"};
	char* drv_type[] = {"B", "A", "C", "D"};
	static u32 last_clock[4] = {0};
	u32 temp;
	s32 err;

	/* Set the power state */
	switch (ios->power_mode) {
	case MMC_POWER_ON:
		break;

	case MMC_POWER_UP:
		err = clk_enable(host->clk_ahb);
		if (err) {
			dev_err(&host->pdev->dev,
				"Failed to enable AHB clock\n");
			return;
		}

		err = clk_enable(host->clk_mod);
		if (err) {
			dev_err(&host->pdev->dev,
				"Failed to enable module clock\n");
			return;
		}

		mdelay(1);

		sw_mci_init_host(mmc);
		enable_irq(host->irq);

		dev_dbg(&host->pdev->dev, "MMC Host powered on\n");
		break;

	case MMC_POWER_OFF:
		SMC_DBG(smc_host, "sdc%d power off !!\n", smc_host->pdev->id);
		disable_irq(smc_host->irq);
		sw_mci_exit_host(smc_host);
		err = clk_reset(smc_host->mclk, AW_CCU_CLK_RESET);
		if (err) {
			SMC_ERR(smc_host, "Failed to set sdc%d reset\n",
				smc_host->pdev->id);
		}
		clk_disable(smc_host->mclk);
		clk_disable(smc_host->hclk);
		sw_mci_hold_io(smc_host);
		smc_host->power_on = 0;
		smc_host->ferror = 0;
		last_clock[id] = 0;

		break;
	}

	/* set bus width */
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		mci_writel(smc_host, REG_WIDTH, SDXC_WIDTH1);
		smc_host->bus_width = 1;
		break;
	case MMC_BUS_WIDTH_4:
		mci_writel(smc_host, REG_WIDTH, SDXC_WIDTH4);
		smc_host->bus_width = 4;
		break;
	case MMC_BUS_WIDTH_8:
		mci_writel(smc_host, REG_WIDTH, SDXC_WIDTH8);
		smc_host->bus_width = 8;
		break;
	}

	/* set ddr mode */
	temp = mci_readl(smc_host, REG_GCTRL);
	if (ios->timing == MMC_TIMING_UHS_DDR50) {
		temp |= SDXC_DDR_MODE;
		smc_host->ddr = 1;
		/* change io driving */
		sw_mci_update_io_driving(smc_host, 3);
	} else {
		temp &= ~SDXC_DDR_MODE;
		smc_host->ddr = 0;
		sw_mci_update_io_driving(smc_host, -1);
	}
	mci_writel(smc_host, REG_GCTRL, temp);

	/* set up clock */
	if (ios->clock && ios->clock != last_clock[id]) {
		if (smc_host->ddr)
			ios->clock = smc_host->pdata->f_ddr_max;
		/* 8bit ddr, mod_clk = 2 * card_clk */
		if (smc_host->ddr && smc_host->bus_width == 8)
			smc_host->mod_clk = ios->clock << 1;
		else
			smc_host->mod_clk = ios->clock;
		smc_host->card_clk = ios->clock;
#ifdef MMC_FPGA
		smc_host->mod_clk = 24000000;
		if (smc_host->card_clk > smc_host->mod_clk)
			smc_host->card_clk = smc_host->mod_clk;
#endif
		sw_mci_set_clk(smc_host, smc_host->card_clk);
		last_clock[id] = ios->clock;
		usleep_range(50000, 55000);
	} else if (!ios->clock) {
		last_clock[id] = 0;
		sw_mci_update_clk(smc_host);
	}
}

static void sw_mci_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	unsigned long flags;
	u32 imask;

	spin_lock_irqsave(&smc_host->lock, flags);
	imask = mci_readl(smc_host, REG_IMASK);
	if (enable)
		imask |= SDXC_SDIOInt;
	else
		imask &= ~SDXC_SDIOInt;
	mci_writel(smc_host, REG_IMASK, imask);
	spin_unlock_irqrestore(&smc_host->lock, flags);
}

void sw_mci_hw_reset(struct mmc_host *mmc)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	u32 id = smc_host->pdev->id;

	if (id == 2 || id == 3) {
		mci_writel(smc_host, REG_HWRST, 0);
		udelay(10);
		mci_writel(smc_host, REG_HWRST, 1);
		udelay(300);
	}
}

static void sw_mci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	struct mmc_command* cmd = mrq->cmd;
	struct mmc_data* data = mrq->data;
	u32 byte_cnt = 0;
	int ret;

	if (sw_mci_card_present(mmc) == 0 || smc_host->ferror || !smc_host->power_on) {
		SMC_DBG(smc_host, "no medium present, ferr %d, pwd %d\n",
			    smc_host->ferror, smc_host->power_on);
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	smc_host->mrq = mrq;
	if (data) {
		byte_cnt = data->blksz * data->blocks;
		mci_writel(smc_host, REG_BLKSZ, data->blksz);
		mci_writel(smc_host, REG_BCNTR, byte_cnt);
		ret = sw_mci_prepare_dma(smc_host, data);
		if (ret < 0) {
			SMC_ERR(smc_host, "smc %d prepare DMA failed\n", smc_host->pdev->id);
			cmd->error = ret;
			cmd->data->error = ret;
			smp_wmb();
			mmc_request_done(smc_host->mmc, mrq);
			return;
		}
	}
	sw_mci_send_cmd(smc_host, cmd);
}

static int sw_mci_do_voltage_switch(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);

	if (smc_host->voltage != SDC_WOLTAGE_3V3 &&
			ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		sw_mci_set_vddio(smc_host, SDC_WOLTAGE_3V3);
		/* wait for 5ms */
		usleep_range(1000, 1500);
		smc_host->voltage = SDC_WOLTAGE_3V3;
		return 0;
	} else if (smc_host->voltage != SDC_WOLTAGE_1V8 &&
			(ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180)) {
		u32 data_down;
		/* clock off */
		sw_mci_oclk_onoff(smc_host, 0, 0);
		/* check whether data[3:0] is 0000 */
		data_down = mci_readl(smc_host, REG_STAS);
		if (!(data_down & SDXC_CardPresent)) {
			/* switch voltage of card vdd to 1.8V */
			sw_mci_set_vddio(smc_host, SDC_WOLTAGE_1V8);
			/* the standard defines the time limit is 5ms, here we
			   wait for 8ms to make sure that the card completes the
			   voltage switching */
			usleep_range(8000, 8500);
			/* clock on again */
			sw_mci_oclk_onoff(smc_host, 1, 0);
			/* wait for 1ms */
			usleep_range(2000, 2500);

			/* check whether data[3:0] is 1111 */
			data_down = mci_readl(smc_host, REG_STAS);
			if (data_down & SDXC_CardPresent) {
				u32 rval = mci_readl(smc_host, REG_RINTR);
				if ((rval & SDXC_VolChgDone & SDXC_CmdDone)
						== (SDXC_VolChgDone & SDXC_CmdDone)) {
					smc_host->voltage = SDC_WOLTAGE_1V8;
					mci_writew(smc_host, REG_RINTR,
						SDXC_VolChgDone | SDXC_CmdDone);
					smc_host->voltage_switching = 0;
					return 0;
				}
			}
		}

		/*
		 * If we are here, that means the switch to 1.8V signaling
		 * failed. We power cycle the card, and retry initialization
		 * sequence by setting S18R to 0.
		 */
		usleep_range(5000, 5500);
		sw_mci_set_vddio(smc_host, SDC_WOLTAGE_OFF);
		usleep_range(1000, 1500);
		sw_mci_set_vddio(smc_host, SDC_WOLTAGE_3V3);
		SMC_ERR(smc_host, ": Switching to 1.8V signalling "
			"voltage failed, retrying with S18R set to 0\n");
		mci_writel(smc_host, REG_GCTRL, mci_readl(smc_host, REG_GCTRL)|SDXC_HWReset);
		mci_writew(smc_host, REG_RINTR, SDXC_VolChgDone | SDXC_CmdDone);
		sw_mci_oclk_onoff(smc_host, 1, 0);
		smc_host->voltage_switching = 0;
		return -EAGAIN;
	} else
		return 0;
}

/*
 * Here we execute a tuning operation to find the sample window of MMC host.
 * Then we select the best sampling point in the host for DDR50, SDR50, and
 * SDR104 modes.
 */
static int sw_mci_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	static const char tuning_blk_4b[] = {
		0xff, 0x0f, 0xff, 0x00, 0xff, 0xcc, 0xc3, 0xcc,
		0xc3, 0x3c, 0xcc, 0xff, 0xfe, 0xff, 0xfe, 0xef,
		0xff, 0xdf, 0xff, 0xdd, 0xff, 0xfb, 0xff, 0xfb,
		0xbf, 0xff, 0x7f, 0xff, 0x77, 0xf7, 0xbd, 0xef,
		0xff, 0xf0, 0xff, 0xf0, 0x0f, 0xfc, 0xcc, 0x3c,
		0xcc, 0x33, 0xcc, 0xcf, 0xff, 0xef, 0xff, 0xee,
		0xff, 0xfd, 0xff, 0xfd, 0xdf, 0xff, 0xbf, 0xff,
		0xbb, 0xff, 0xf7, 0xff, 0xf7, 0x7f, 0x7b, 0xde
	};
	static const char tuning_blk_8b[] = {
		0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00,
		0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc, 0xcc,
		0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff, 0xff,
		0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee, 0xff,
		0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd, 0xdd,
		0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff, 0xbb,
		0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff, 0xff,
		0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee, 0xff,
		0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00,
		0x00, 0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc,
		0xcc, 0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff,
		0xff, 0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee,
		0xff, 0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd,
		0xdd, 0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff,
		0xbb, 0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff,
		0xff, 0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee
	};
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	u32 sample_min = 1;
	u32 sample_max = 0;
	u32 sample_bak = smc_host->sclk_dly;
	u32 sample_dly = 0;
	u32 sample_win = 0;
	u32 loops = 64;
	u32 tuning_done = 0;
	char* rcv_pattern = (char*)kmalloc(128, GFP_KERNEL|GFP_DMA);
	char* std_pattern = NULL;
	int err = 0;

	if (!rcv_pattern) {
		SMC_ERR(smc_host, "sdc%d malloc tuning pattern buffer failed\n",
				smc_host->pdev->id);
		return -EIO;
	}
	SMC_MSG(smc_host, "sdc%d executes tuning operation\n", smc_host->pdev->id);
	/*
	 * The Host Controller needs tuning only in case of SDR104 mode
	 * and for SDR50 mode. Issue CMD19 repeatedly till get all of the
	 * sample points or the number of loops reaches 40 times or a
	 * timeout of 150ms occurs.
	 */
	do {
		struct mmc_command cmd = {0};
		struct mmc_data data = {0};
		struct mmc_request mrq = {0};
		struct scatterlist sg;

		sw_mci_set_clk_dly(smc_host, smc_host->oclk_dly, sample_dly);
		cmd.opcode = opcode;
		cmd.arg = 0;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
		if (opcode == MMC_SEND_TUNING_BLOCK_HS200) {
			if (mmc->ios.bus_width == MMC_BUS_WIDTH_8) {
				sg.length = 128;
				data.blksz = 128;
				std_pattern = (char*)tuning_blk_8b;
			} else if (mmc->ios.bus_width == MMC_BUS_WIDTH_4) {
				sg.length = 64;
				data.blksz = 64;
				std_pattern = (char*)tuning_blk_4b;
			}
		} else {
			sg.length = 64;
			data.blksz = 64;
			std_pattern = (char*)tuning_blk_4b;
		}
		data.blocks = 1;
		data.flags = MMC_DATA_READ;
		data.sg = &sg;
		data.sg_len = 1;
		sg_init_one(&sg, rcv_pattern, sg.length);

		mrq.cmd = &cmd;
		mrq.data = &data;

		mmc_wait_for_req(mmc, &mrq);
		/*
		 * If no error happened in the transmission, compare data with
		 * the tuning pattern. If there is no error, record the minimal
		 * and the maximal value of the sampling clock delay to find
		 * the best sampling point in the sampling window.
		 */
		if (!cmd.error && !data.error) {
			if (!memcmp(rcv_pattern, std_pattern, data.blksz)) {
				SMC_MSG(smc_host, "sdc%d tuning ok, sclk_dly %d\n",
					smc_host->pdev->id, sample_dly);
				if (!sample_win)
					sample_min = sample_dly;
				sample_win++;
				if (sample_dly == 7) {
					SMC_MSG(smc_host, "sdc%d tuning reach to max sclk_dly 7\n",
						smc_host->pdev->id);
					tuning_done = 1;
					sample_max = sample_dly;
					break;
				}
			} else if (sample_win) {
				SMC_MSG(smc_host, "sdc%d tuning data failed, sclk_dly %d\n",
					smc_host->pdev->id, sample_dly);
				tuning_done = 1;
				sample_max = sample_dly-1;
				break;
			}
		} else if (sample_win) {
			SMC_MSG(smc_host, "sdc%d tuning trans fail, sclk_dly %d\n",
				smc_host->pdev->id, sample_dly);
			tuning_done = 1;
			sample_max = sample_dly-1;
			break;
		}
		sample_dly++;
		/* if sclk_dly reach to 7(maximum), down the clock and tuning again */
		if (sample_dly == 8 && loops)
			break;
	} while (!tuning_done && loops--);

	/* select the best sampling point from the sampling window */
	if (sample_win) {
		sample_dly = sample_min + sample_win/2;
		SMC_MSG(smc_host, "sdc%d sample_window:[%d, %d], sample_point %d\n",
				smc_host->pdev->id, sample_min, sample_max, sample_dly);
		sw_mci_set_clk_dly(smc_host, smc_host->oclk_dly, sample_dly);
		err = 0;
	} else {
		SMC_ERR(smc_host, "sdc%d cannot find a sample point\n", smc_host->pdev->id);
		sw_mci_set_clk_dly(smc_host, smc_host->oclk_dly, sample_bak);
		mmc->ios.bus_width = MMC_BUS_WIDTH_1;
		mmc->ios.timing = MMC_TIMING_LEGACY;
		err = -EIO;
	}

	kfree(rcv_pattern);
	return err;
}

/*
 * Here provide a function to scan card, for some SDIO cards that
 * may stay in busy status after writing operations. MMC host does
 * not wait for ready itself. So the driver of this kind of cards
 * should call this function to check the real status of the card.
 */
void sw_mci_rescan_card(unsigned id, unsigned insert)
{
	struct sunxi_mmc_host *smc_host = NULL;

	BUG_ON(id > 3);
	BUG_ON(sw_host[id] == NULL);
	smc_host = sw_host[id];

	smc_host->present = insert ? 1 : 0;
	mmc_detect_change(smc_host->mmc, 0);
	return;
}
EXPORT_SYMBOL_GPL(sw_mci_rescan_card);

int sw_mci_check_r1_ready(struct mmc_host* mmc, unsigned ms)
{
	struct sunxi_mmc_host *smc_host = mmc_priv(mmc);
	unsigned expire = jiffies + msecs_to_jiffies(ms);
	do {
		if (!(mci_readl(smc_host, REG_STAS) & SDXC_CardDataBusy))
			break;
	} while (jiffies < expire);

	if ((mci_readl(smc_host, REG_STAS) & SDXC_CardDataBusy)) {
		SMC_MSG(smc_host, "wait r1 rdy %d ms timeout\n", ms);
		return -1;
	} else
		return 0;
}
EXPORT_SYMBOL_GPL(sw_mci_check_r1_ready);

static struct mmc_host_ops sw_mci_ops = {
	.request	= sw_mci_request,
	.set_ios	= sw_mci_set_ios,
	.get_ro		= sw_mci_get_ro,
	.get_cd		= sw_mci_card_present,
	.enable_sdio_irq= sw_mci_enable_sdio_irq,
	.hw_reset	= sw_mci_hw_reset,
	.start_signal_voltage_switch = sw_mci_do_voltage_switch,
	.execute_tuning = sw_mci_execute_tuning,
};

static int __init sunxi_mci_probe(struct platform_device *pdev)
{
	struct sunxi_mmc_host *smc_host = NULL;
	struct mmc_host	*mmc = NULL;
	int ret = 0;

	mmc = mmc_alloc_host(sizeof(struct sunxi_mmc_host), &pdev->dev);
	if (!mmc) {
		SMC_ERR(smc_host, "mmc alloc host failed\n");
		ret = -ENOMEM;
		goto probe_out;
	}

	smc_host = mmc_priv(mmc);
	memset((void*)smc_host, 0, sizeof(smc_host));
	smc_host->mmc	= mmc;
	smc_host->pdev	= pdev;
	smc_host->pdata	= pdev->dev.platform_data;
//	smc_host->cd_mode = smc_host->pdata->cdmode;
//	smc_host->debuglevel = CONFIG_MMC_PRE_DBGLVL_SUNXI;

	spin_lock_init(&smc_host->lock);
	tasklet_init(&smc_host->tasklet, sw_mci_tasklet, (unsigned long) smc_host);

	if (sw_mci_resource_request(smc_host)) {
		SMC_ERR(smc_host, "%s: Failed to get resouce.\n", dev_name(&pdev->dev));
		goto probe_free_host;
	}

//	smc_host->irq = SMC_IRQNO(pdev->id);
	if (request_irq(smc_host->irq, sw_mci_irq, 0, DRIVER_NAME, smc_host)) {
		SMC_ERR(smc_host, "Failed to request smc card interrupt.\n");
		ret = -ENOENT;
		goto probe_free_resource;
	}
	disable_irq(smc_host->irq);

	if (smc_host->cd_mode == CARD_ALWAYS_PRESENT) {
		smc_host->present = 1;
	} else if (smc_host->cd_mode == CARD_DETECT_BY_GPIO_IRQ) {
		u32 cd_hdle;
//		cd_hdle = sw_gpio_irq_request(smc_host->pdata->cd.gpio, TRIG_EDGE_DOUBLE,
//					&sw_mci_cd_irq, smc_host);
		if (!cd_hdle) {
			SMC_ERR(smc_host, "Failed to get gpio irq for card detection\n");
		}
		smc_host->cd_hdle = cd_hdle;
//		smc_host->present = !__gpio_get_value(smc_host->pdata->cd.gpio);
	} else if (smc_host->cd_mode == CARD_DETECT_BY_GPIO_POLL) {
		init_timer(&smc_host->cd_timer);
		smc_host->cd_timer.expires = jiffies + 1*HZ;
		smc_host->cd_timer.function = &sw_mci_cd_cb;
		smc_host->cd_timer.data = (unsigned long)smc_host;
		add_timer(&smc_host->cd_timer);
		smc_host->present = 0;
	}

	mmc->ops        = &sw_mci_ops;
//	mmc->ocr_avail	= smc_host->pdata->ocr_avail;
//	mmc->caps	= smc_host->pdata->caps;
//	mmc->caps2	= smc_host->pdata->caps2;
//	mmc->pm_caps	= MMC_PM_KEEP_POWER|MMC_PM_WAKE_SDIO_IRQ;
//	mmc->f_min	= smc_host->pdata->f_min;
//	mmc->f_max      = smc_host->pdata->f_max;
	mmc->max_blk_count	= 8192;
	mmc->max_blk_size	= 4096;
	mmc->max_req_size	= mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size	= mmc->max_req_size;
	mmc->max_segs	    	= 128;

	ret = mmc_add_host(mmc);
	if (ret) {
		SMC_ERR(smc_host, "Failed to add mmc host.\n");
		goto probe_free_irq;
	}
	platform_set_drvdata(pdev, mmc);

	SMC_MSG(smc_host, "sdc%d Probe: base:0x%p irq:%u sg_cpu:%p(%x) ret %d.\n",
		pdev->id, smc_host->reg_base, smc_host->irq,
		smc_host->sg_cpu, smc_host->sg_dma, ret);

	goto probe_out;

probe_free_irq:
	if (smc_host->irq)
		free_irq(smc_host->irq, smc_host);
probe_free_resource:
	sw_mci_resource_release(smc_host);
probe_free_host:
	mmc_free_host(mmc);
probe_out:
	return ret;
}

static int __exit sunxi_mci_remove(struct platform_device *pdev)
{
	struct mmc_host    	*mmc  = platform_get_drvdata(pdev);
	struct sunxi_mmc_host	*smc_host = mmc_priv(mmc);

	SMC_MSG(smc_host, "%s: Remove.\n", dev_name(&pdev->dev));

	sw_mci_exit_host(smc_host);

	mmc_remove_host(mmc);

	tasklet_disable(&smc_host->tasklet);
	free_irq(smc_host->irq, smc_host);
	if (smc_host->cd_mode == CARD_DETECT_BY_GPIO_POLL)
		del_timer(&smc_host->cd_timer);
//	else if (smc_host->cd_mode == CARD_DETECT_BY_GPIO_IRQ)
//		sw_gpio_irq_free(smc_host->cd_hdle);

	sw_mci_resource_release(smc_host);

	mmc_free_host(mmc);

	return 0;
}

static struct platform_driver sunxi_mci_driver = {
	.driver = {
		.name	= "sunxi-mci",
		.owner	= THIS_MODULE,
	},
	.probe		= sunxi_mci_probe,
	.remove		= __exit_p(sunxi_mci_remove),
};
module_platform_driver(sunxi_mci_driver);

MODULE_DESCRIPTION("Winner's SD/MMC Card Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aaron.maoye<leafy.myeh@reuuimllatech.com>");
MODULE_ALIAS("platform:sunxi-mmc");
