// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * CHUB IF Driver Debug
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 * Authors:
 *      Boojin Kim <boojin.kim@samsung.com>
 *      Sukwon Ryu <sw.ryoo@samsung.com>
 *
 */

#include <linux/version.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/iommu.h>
#include <linux/of_reserved_mem.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <soc/samsung/cal-if.h>
#include "chub_dbg.h"
#include "ipc_chub.h"
#include "chub.h"
#ifdef CONFIG_SENSOR_DRV
#include "main.h"
#endif

#define NUM_OF_GPR (17)
#define GPR_PC_INDEX (16)
#define AREA_NAME_MAX (8)
/* it's align ramdump side to prevent override */
#define SRAM_ALIGN (1024 * 8)
#define S_IRWUG (0660)
#define HARDFAULT_OFFSET 0xFF000

#ifdef CONFIG_SENSORS_SSP
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0))
#include <linux/time64.h>
#define SEC_TIMESPEC timespec64
#define SEC_GETTIMEOFDAY ktime_get_real_ts64
#define SEC_RTC_TIME_TO_TM rtc_time64_to_tm
#else
#include <linux/time.h>
#define SEC_TIMESPEC timeval
#define SEC_GETTIMEOFDAY do_gettimeofday
#define SEC_RTC_TIME_TO_TM rtc_time_to_tm
#endif
#endif

struct map_info {
	char name[AREA_NAME_MAX];
	u32 offset;
	u32 size;
};

struct dbg_dump {
	union {
		struct {
			struct map_info info[DBG_AREA_MAX];
			uint64_t time;
			int reason;
			struct contexthub_ipc_info chub;
			struct ipc_area ipc_addr[IPC_REG_MAX];
			u32 gpr[NUM_OF_GPR];
			u32 cm55_cur;
			u32 cm55_secure;
			u32 cm55_nonsecure;
			u32 cm55_ctrl;
			u32 cm55_fault[2];
			u32 cm55_user[5];
			u32 cmu[CMU_REG_MAX];
			u32 sys[SYS_REG_MAX];
			u32 wdt[WDT_REG_MAX];
			u32 timer[TIMER_REG_MAX];
			u32 pwm[PWM_REG_MAX];
			u32 rtc[RTC_REG_MAX];
			u32 usi[USI_REG_MAX * MAX_USI_CNT];
#ifdef CONFIG_SENSORS_SSP
			char dumptime[30];
#endif
		};
		char dummy[SRAM_ALIGN];
	};
	char sram[];
};

struct RamPersistedDataAndDropbox {
	u32 magic;
	u32 r[16];
	u32 sr_hfsr_cfsr_lo;
	u32 bits;
	u32 tid;
};

typedef struct
{
	uint32_t CPUID;    /*!< Offset: 0x000 (R/ )  CPUID Base Register */
	uint32_t ICSR;     /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register*/
	uint32_t VTOR;     /*!< Offset: 0x008 (R/W)  Vector Table Offset Register        */
	uint32_t AIRCR;    /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
	uint32_t SCR;      /*!< Offset: 0x010 (R/W)  System Control Register             */
	uint32_t CCR;      /*!< Offset: 0x014 (R/W)  Configuration Control Register      */
	uint8_t  SHP[12];  /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
	uint32_t SHCSR;    /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
	uint32_t CFSR;     /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register  */
	uint32_t HFSR;     /*!< Offset: 0x02C (R/W)  HardFault Status Register           */
	uint32_t DFSR;     /*!< Offset: 0x030 (R/W)  Debug Fault Status Register         */
	uint32_t MMFAR;    /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register    */
	uint32_t BFAR;     /*!< Offset: 0x038 (R/W)  BusFault Address Register           */
	uint32_t AFSR;     /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register     */
	uint32_t PFR[2];   /*!< Offset: 0x040 (R/ )  Processor Feature Register          */
	uint32_t DFR;      /*!< Offset: 0x048 (R/ )  Debug Feature Register              */
	uint32_t ADR;      /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register          */
	uint32_t MMFR[4];  /*!< Offset: 0x050 (R/ )  Memory Model Feature Register       */
	uint32_t ISAR[5];  /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
	uint32_t RESERVED0[5];
	uint32_t CPACR;    /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;

struct hardFaultDebugInfo {
	uint32_t msp;
	uint32_t psp;
	uint32_t sp;
	uint32_t reserved0;
	uint32_t NVIC_ISER;
	uint32_t NVIC_ICER;
	uint32_t NVIC_ISPR;
	uint32_t NVIC_ICPR;
	uint32_t NVIC_IABR;
	uint32_t NVIC_IP;
	uint32_t NVIC_STIR;
	uint32_t reserved1;
	//SCB_Type scb;
	uint32_t reserved2;
};

static struct dbg_dump *p_dbg_dump;
static struct reserved_mem *chub_rmem;
struct hardFaultDebugInfo hardFaultInfo;
#if IS_ENABLED(CONFIG_SEC_PM_DEBUG)
static struct ipc_cpu_info sec_pm_chub_cpu;
#endif

static void *get_contexthub_info_from_dev(struct device *dev) {
#ifdef CONFIG_SENSOR_DRV
	struct nanohub_data *data = dev_get_nanohub_data(dev);

	return data->pdata->mailbox_client;
#else
	 return dev_get_drvdata(dev);
#endif
}

static void chub_dbg_dump_gpr(struct contexthub_ipc_info *chub)
{
	if (p_dbg_dump) {
		int i;
		struct dbg_dump *p_dump = p_dbg_dump;

		IPC_HW_WRITE_DUMPGPR_CTRL(chub->chub_dumpgpr, 0x1);
		/* dump GPR */
		for (i = 0; i <= GPR_PC_INDEX - 1; i++)
			p_dump->gpr[i] =
			    readl(chub->chub_dumpgpr + REG_CHUB_DUMPGPR_GP0R +
				  i * 4);
		p_dump->gpr[GPR_PC_INDEX] =
		    readl(chub->chub_dumpgpr + REG_CHUB_DUMPGPR_PCR);

		nanohub_dev_info(chub->dev, "%s: r0:0x%x, r1:0x%x, r2:0x%x, r3:0x%x, r4:0x%x, r5:0x%x, r6:0x%x, r7:0x%x\n",
				 __func__, p_dump->gpr[0], p_dump->gpr[1], p_dump->gpr[2],
				 p_dump->gpr[3], p_dump->gpr[4], p_dump->gpr[5], p_dump->gpr[6],
				 p_dump->gpr[7]);

		nanohub_dev_info(chub->dev, "%s: r8:0x%x, r9:0x%x, r10:0x%x, r11:0x%x, r12:0x%x, sp:0x%x, lr:0x%x, r15:0x%x, pc:0x%x\n",
				 __func__, p_dump->gpr[8], p_dump->gpr[9], p_dump->gpr[10],
				 p_dump->gpr[11], p_dump->gpr[12], p_dump->gpr[13], p_dump->gpr[14],
				 p_dump->gpr[15], p_dump->gpr[16]);

#if IS_ENABLED(CONFIG_EXYNOS_LPD)
		if (chub->sysreg_chub) {
			p_dump->cm55_cur = __raw_readl(chub->sysreg_chub + YAMIN_INST_CUR);
			p_dump->cm55_secure = __raw_readl(chub->sysreg_chub + YAMIN_SECURE) & 0x1ffffff;
			p_dump->cm55_nonsecure = __raw_readl(chub->sysreg_chub + YAMIN_NONSECURE) & 0x1ffffff;
			p_dump->cm55_ctrl = __raw_readl(chub->sysreg_chub + YAMIN_INST_CTRL) & 0xf;
			p_dump->cm55_fault[0] = __raw_readl(chub->sysreg_chub + YAMIN_INST_FAULT0) & 0x3ff;
			p_dump->cm55_fault[1] = __raw_readl(chub->sysreg_chub + YAMIN_INST_FAULT1);
			for (i = 0 ; i < 5 ; i++) {
				p_dump->cm55_user[i] = __raw_readl(chub->sysreg_chub + i * 4);
			}
			nanohub_dev_info(chub->dev, "========================================\n");
			nanohub_dev_info(chub->dev, "CM55 register dump\n");
			nanohub_dev_info(chub->dev, "INST_CUR   : 0x%x\n",
					 p_dbg_dump->cm55_cur);
			nanohub_dev_info(chub->dev, "INST_CTRL  : 0x%x\n",
					 p_dbg_dump->cm55_ctrl);
			nanohub_dev_info(chub->dev, "INST_FAULT : 0x%x 0x%x\n",
					 p_dbg_dump->cm55_fault[0],
					 p_dbg_dump->cm55_fault[1]);
			nanohub_dev_info(chub->dev, "USER       : 0x%x 0x%x 0x%x 0x%x 0x%x\n",
					p_dbg_dump->cm55_user[0], p_dbg_dump->cm55_user[1],
					p_dbg_dump->cm55_user[2], p_dbg_dump->cm55_user[3],
					p_dbg_dump->cm55_user[4]);
			nanohub_dev_info(chub->dev, "========================================\n");
		}
#endif
	}
}

static void chub_dbg_dump_cmu(struct contexthub_ipc_info *chub)
{
	if (p_dbg_dump && !IS_ERR_OR_NULL(chub->chub_dump_cmu)) {
		int i;
		struct dbg_dump *p_dump = p_dbg_dump;

		/* dump CMU */
		for (i = 0; i <= CMU_REG_MAX - 1; i++)
			p_dump->cmu[i] =
			    readl(chub->chub_dump_cmu +
			    dump_chub_cmu_registers[i]);

		nanohub_dev_info(chub->dev, "%s: r0:0x%x, r1:0x%x, r2:0x%x, r3:0x%x, r4:0x%x, r5:0x%x, r6:0x%x, r7:0x%x\n",
				 __func__, p_dump->cmu[0], p_dump->cmu[1], p_dump->cmu[2],
				 p_dump->cmu[3], p_dump->cmu[4], p_dump->cmu[5],
				 p_dump->cmu[6], p_dump->cmu[7]);

		nanohub_dev_info(chub->dev, "%s: r8:0x%x, r9:0x%x, r10:0x%x, r11:0x%x, r12:0x%x, r13:0x%x, r14:0x%x, r15:0x%x\n",
				 __func__, p_dump->cmu[8], p_dump->cmu[9], p_dump->cmu[10],
				 p_dump->cmu[11], p_dump->cmu[12], p_dump->cmu[13],
				 p_dump->cmu[14], p_dump->cmu[15]);

		nanohub_dev_info(chub->dev, "%s: r16:0x%x\n", __func__, p_dump->cmu[16]);
	}
}

static void chub_dbg_dump_sys(struct contexthub_ipc_info *chub)
{
	if (p_dbg_dump && !IS_ERR_OR_NULL(chub->chub_dump_sys)) {
		int i;
		struct dbg_dump *p_dump = p_dbg_dump;

		/* dump SYS */
		for (i = 0; i <= SYS_REG_MAX - 1; i++)
			p_dump->sys[i] =
			    readl(chub->chub_dump_sys +
			    dump_chub_sys_registers[i]);

		nanohub_dev_info(chub->dev, "%s: r0:0x%x, r1:0x%x, r2:0x%x, r3:0x%x, r4:0x%x, r5:0x%x, r6:0x%x, r7:0x%x\n",
				 __func__, p_dump->sys[0], p_dump->sys[1], p_dump->sys[2],
				 p_dump->sys[3], p_dump->sys[4], p_dump->sys[5],
				 p_dump->sys[6], p_dump->sys[7]);

		nanohub_dev_info(chub->dev, "%s: r8:0x%x, r9:0x%x, r10:0x%x, r11:0x%x, r12:0x%x, r13:0x%x, r14:0x%x, r15:0x%x\n",
				 __func__, p_dump->sys[8], p_dump->sys[9],
				 p_dump->sys[10], p_dump->sys[11], p_dump->sys[12],
				 p_dump->sys[13], p_dump->sys[14], p_dump->sys[15]);

		nanohub_dev_info(chub->dev, "%s: r16:0x%x, r17:0x%x, r18:0x%x, r19:0x%x, r20:0x%x\n",
				 __func__, p_dump->sys[16], p_dump->sys[17], p_dump->sys[18],
				 p_dump->sys[19], p_dump->sys[20]);

	}
}

static void chub_dbg_dump_wdt(struct contexthub_ipc_info *chub)
{
	if (p_dbg_dump && !IS_ERR_OR_NULL(chub->chub_dump_wdt)) {
		int i;
		struct dbg_dump *p_dump = p_dbg_dump;

		/* dump wdt */
		for (i = 0; i <= WDT_REG_MAX - 1; i++)
			p_dump->wdt[i] =
			    readl(chub->chub_dump_wdt + dump_chub_wdt_registers[i]);

		nanohub_dev_info(chub->dev, "%s: r0:0x%x, r1:0x%x\n",
				 __func__, p_dump->wdt[0], p_dump->wdt[1]);
	}
}

static void chub_dbg_dump_timer(struct contexthub_ipc_info *chub)
{
	if (p_dbg_dump && !IS_ERR_OR_NULL(chub->chub_dump_wdt)) {
		int i;
		struct dbg_dump *p_dump = p_dbg_dump;

		/* dump timer */
		for (i = 0; i <= TIMER_REG_MAX - 1; i++)
			p_dump->timer[i] =
			    readl(chub->chub_dump_timer +
			    dump_chub_timer_registers[i]);

		nanohub_dev_info(chub->dev, "%s: r0:0x%x, r1:0x%x, r2:0x%x\n",
				 __func__, p_dump->timer[0], p_dump->timer[1], p_dump->timer[2]);
	}
}

static void chub_dbg_dump_pwm(struct contexthub_ipc_info *chub)
{
	if (p_dbg_dump && !IS_ERR_OR_NULL(chub->chub_dump_pwm)) {
		int i;
		struct dbg_dump *p_dump = p_dbg_dump;

		/* dump pwm */
		for (i = 0; i <= PWM_REG_MAX - 1; i++)
			p_dump->pwm[i] =
				readl(chub->chub_dump_pwm + dump_chub_pwm_registers[i]);

		nanohub_dev_info(chub->dev, "%s: r0:0x%x, r1:0x%x, r2:0x%x, r3:0x%x, r4:0x%x, r5:0x%x, r6:0x%x, r7:0x%x\n",
				 __func__, p_dump->pwm[0],
				 p_dump->pwm[1], p_dump->pwm[2],
				 p_dump->pwm[3], p_dump->pwm[4],
				 p_dump->pwm[5], p_dump->pwm[6], p_dump->pwm[7]);

		nanohub_dev_info(chub->dev, "%s: r8:0x%x, r9:0x%x, r10:0x%x, r11:0x%x, r12:0x%x, r13:0x%x, r14:0x%x, r15:0x%x\n",
				 __func__, p_dump->pwm[8],
				 p_dump->pwm[9], p_dump->pwm[10],
				 p_dump->pwm[11], p_dump->pwm[12],
				 p_dump->pwm[13], p_dump->pwm[14], p_dump->pwm[15]);

	}
}

static void chub_dbg_dump_rtc(struct contexthub_ipc_info *chub)
{
	if (p_dbg_dump && !IS_ERR_OR_NULL(chub->chub_dump_rtc)) {
		int i;
		struct dbg_dump *p_dump = p_dbg_dump;

		/* dump rtc */
		for (i = 0; i <= RTC_REG_MAX - 1; i++)
			p_dump->rtc[i] =
				readl(chub->chub_dump_rtc +
				dump_chub_rtc_registers[i]);

		nanohub_dev_info(chub->dev, "%s: r0:0x%x, r1:0x%x, r2:0x%x, r3:0x%x, r4:0x%x, r5:0x%x, r6:0x%x, r7:0x%x\n",
				 __func__, p_dump->rtc[0], p_dump->rtc[1], p_dump->rtc[2],
				 p_dump->rtc[3], p_dump->rtc[4], p_dump->rtc[5], p_dump->rtc[6],
				 p_dump->rtc[7]);

		nanohub_dev_info(chub->dev, "%s: r8:0x%x, r9:0x%x, r10:0x%x, r11:0x%x, r12:0x%x, r13:0x%x, r14:0x%x, r15:0x%x\n",
				 __func__, p_dump->rtc[8],
				 p_dump->rtc[9], p_dump->rtc[10],
				 p_dump->rtc[11], p_dump->rtc[12],
				 p_dump->rtc[13], p_dump->rtc[14],
				 p_dump->rtc[15]);

		nanohub_dev_info(chub->dev, "%s: r16:0x%x, r17:0x%x, r18:0x%x, r19:0x%x, r20:0x%x, r21:0x%x, r22:0x%x\n",
				 __func__, p_dump->rtc[16],
				 p_dump->rtc[17], p_dump->rtc[18],
				 p_dump->rtc[19], p_dump->rtc[20],
				 p_dump->rtc[21], p_dump->rtc[22]);
	}
}

static void chub_dbg_dump_usi(struct contexthub_ipc_info *chub)
{
	if (p_dbg_dump && chub->usi_cnt) {
		int i;
		int j;
		int index = 0;
		int index_tmp = 0;
		u32 usi_protocol;
		struct dbg_dump *p_dump = p_dbg_dump;

		for (j = 0; j < chub->usi_cnt; j++) {
			/* dump usi */
			usi_protocol =
			    READ_CHUB_USI_CONF(chub->usi_array[j]);
			switch (usi_protocol) {
			case USI_PROTOCOL_UART:
				nanohub_dev_info(chub->dev, "%s: chub_usi %d config as UART\n",
						 __func__, j);
				index_tmp = index;
				for (i = 0; i <= UART_REG_MAX - 1; i++) {
					p_dump->usi[index++] =
					   readl(chub->usi_array[j]
						 + dump_chub_uart_registers[i]);
				}

				nanohub_dev_info(chub->dev, "%s: r0:0x%x, r1:0x%x, r2:0x%x, r3:0x%x, r4:0x%x, r5:0x%x, r6:0x%x, r7:0x%x\n",
						 __func__,
						 p_dump->usi[index_tmp],
						 p_dump->usi[index_tmp + 1],
						 p_dump->usi[index_tmp + 2],
						 p_dump->usi[index_tmp + 3],
						 p_dump->usi[index_tmp + 4],
						 p_dump->usi[index_tmp + 5],
						 p_dump->usi[index_tmp + 6],
						 p_dump->usi[index_tmp + 7]);

				nanohub_dev_info(chub->dev, "%s: r8:0x%x, r9:0x%x, r10:0x%x, r11:0x%x, r12:0x%x, r13:0x%x, r14:0x%x, r15:0x%x\n",
						 __func__,
						 p_dump->usi[index_tmp + 8],
						 p_dump->usi[index_tmp + 9],
						 p_dump->usi[index_tmp + 10],
						 p_dump->usi[index_tmp + 11],
						 p_dump->usi[index_tmp + 12],
						 p_dump->usi[index_tmp + 13],
						 p_dump->usi[index_tmp + 14],
						 p_dump->usi[index_tmp + 15]);
				break;
			case USI_PROTOCOL_SPI:
				nanohub_dev_info(chub->dev, "%s: chub_usi %d config as SPI\n",
						 __func__, j);
				index_tmp = index;
				for (i = 0; i <= SPI_REG_MAX - 1; i++) {
					p_dump->usi[index++] =
					    readl(chub->usi_array[j]
						  + dump_chub_spi_registers[i]);
				}

				nanohub_dev_info(chub->dev, "%s: r0:0x%x, r1:0x%x, r2:0x%x, r3:0x%x, r4:0x%x, r5:0x%x, r6:0x%x, r7:0x%x\n",
						 __func__,
						 p_dump->usi[index_tmp],
						 p_dump->usi[index_tmp + 1],
						 p_dump->usi[index_tmp + 2],
						 p_dump->usi[index_tmp + 3],
						 p_dump->usi[index_tmp + 4],
						 p_dump->usi[index_tmp + 5],
						 p_dump->usi[index_tmp + 6],
						 p_dump->usi[index_tmp + 7]);

				nanohub_dev_info(chub->dev, "%s: r8:0x%x, r9:0x%x, r10:0x%x\n",
						 __func__,
						 p_dump->usi[index_tmp + 8],
						 p_dump->usi[index_tmp + 9],
						 p_dump->usi[index_tmp + 10]);
				break;
			case USI_PROTOCOL_I2C:
				nanohub_dev_info(chub->dev,
						 "%s: chub_usi %d config as I2C\n", __func__, j);
				index_tmp = index;
				for (i = 0; i <= I2C_REG_MAX - 1; i++)
					p_dump->usi[index++] =
					   readl(chub->usi_array[j] + dump_chub_i2c_registers[i]);

				nanohub_dev_info(chub->dev, "%s: r0:0x%x, r1:0x%x, r2:0x%x, r3:0x%x, r4:0x%x, r5:0x%x, r6:0x%x, r7:0x%x\n",
						 __func__,
						 p_dump->usi[index_tmp],
						 p_dump->usi[index_tmp + 1],
						 p_dump->usi[index_tmp + 2],
						 p_dump->usi[index_tmp + 3],
						 p_dump->usi[index_tmp + 4],
						 p_dump->usi[index_tmp + 5],
						 p_dump->usi[index_tmp + 6],
						 p_dump->usi[index_tmp + 7]);

				nanohub_dev_info(chub->dev, "%s: r8:0x%x, r9:0x%x, r10:0x%x, r11:0x%x, r12:0x%x, r13:0x%x, r14:0x%x, r15:0x%x\n",
						 __func__,
						 p_dump->usi[index_tmp + 8],
						 p_dump->usi[index_tmp + 9],
						 p_dump->usi[index_tmp + 10],
						 p_dump->usi[index_tmp + 11],
						 p_dump->usi[index_tmp + 12],
						 p_dump->usi[index_tmp + 13],
						 p_dump->usi[index_tmp + 14],
						 p_dump->usi[index_tmp + 15]);

				nanohub_dev_info(chub->dev, "%s: r16:0x%x, r17:0x%x, r18:0x%x, r19:0x%x\n",
						 __func__,
						 p_dump->usi[index_tmp + 16],
						 p_dump->usi[index_tmp + 17],
						 p_dump->usi[index_tmp + 18],
						 p_dump->usi[index_tmp + 19]);
				break;
			default:
				break;
			}
		}
	}
}

static u32 get_dbg_dump_size(void)
{
	return sizeof(struct dbg_dump) + ipc_get_chub_mem_size();
};

static u32 get_chub_dumped_registers(int cnt)
{
	return sizeof(u32) * (NUM_OF_GPR + CMU_REG_MAX + SYS_REG_MAX + WDT_REG_MAX +
		 TIMER_REG_MAX + PWM_REG_MAX + RTC_REG_MAX + USI_REG_MAX * cnt);
};

void chub_dbg_register_dump_to_dss(uint32_t sram_phys, uint32_t sram_size)
{
    if (!chub_rmem) {
        pr_info("chub_rmem not available. skip register dram dump\n");
    } else {
        pr_info("register chub_dram to dss\n");
        pr_info("chub_dram info : base : 0x%X, size : 0x%X\n", chub_rmem->base, chub_rmem->size);
        dbg_snapshot_add_bl_item_info("chub_dram", chub_rmem->base, chub_rmem->size);
    }
}

phys_addr_t get_chub_rmem_size(void)
{
	return chub_rmem->size;
}

void *get_chub_dram_ptr(void)
{
    return (void *)p_dbg_dump;
}

#ifdef CONFIG_SENSORS_SSP
void set_chub_dump_time(void)
{
	struct dbg_dump *p_dump = p_dbg_dump;
	struct SEC_TIMESPEC tv;
	struct rtc_time tm;
	unsigned long local_time;
	int size = 0;

	/* Format the Log time R#: [hr:min:sec.microsec] */
	SEC_GETTIMEOFDAY(&tv);
	/* Convert rtc to local time */
	local_time = (u32)(tv.tv_sec - (sys_tz.tz_minuteswest * 60));
	SEC_RTC_TIME_TO_TM(local_time, &tm);

	size = sprintf(p_dump->dumptime, "[%d-%2d-%2d %02d:%02d:%02d](%d)",
		  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		  tm.tm_hour, tm.tm_min, tm.tm_sec, sys_tz.tz_minuteswest);

	nanohub_debug("%s (%d)\n", p_dump->dumptime, size);
}
#endif

// return 0 if copying hardFaultInfo success.
// return -1 if failed.(p_dbg_dump is null)
int chub_dbg_get_hardFaultInfo(struct contexthub_ipc_info *chub) 
{
	if (p_dbg_dump) {
		memset(&hardFaultInfo, 0, sizeof(struct hardFaultDebugInfo));
		memcpy(&hardFaultInfo, p_dbg_dump->sram + HARDFAULT_OFFSET, sizeof(struct hardFaultDebugInfo));
		memset_io(chub->sram + HARDFAULT_OFFSET, 0, sizeof(struct hardFaultDebugInfo));
		return 0;
	} else {
		return -1;
	}
}

#define CHUB_LPD_OFFSET (SZ_1M + SZ_512K + SZ_256K)
/* dump hw into dram (chub reserved mem) */
void chub_dbg_dump_ram(enum chub_err_type reason)
{
	if (p_dbg_dump) {
		p_dbg_dump->time = sched_clock();
		p_dbg_dump->reason = reason;

		/* dump SRAM to reserved DRAM */
		memcpy_fromio(p_dbg_dump->sram,
			      ipc_get_base(IPC_REG_DUMP), ipc_get_chub_mem_size());
		memcpy_fromio(p_dbg_dump->sram + CHUB_LPD_OFFSET,
			ipc_get_base(IPC_REG_DUMP) + CHUB_LPD_OFFSET, SZ_256K);
		
		if (reason == CHUB_ERR_KERNEL_PANIC)
			chub_dbg_dump_gpr(&p_dbg_dump->chub);

#ifdef CONFIG_SENSORS_SSP
		set_chub_dump_time();
#endif
	}
}

static void chub_dbg_dump_status(struct contexthub_ipc_info *chub)
{
	int i;

#ifdef CONFIG_SENSOR_DRV
	struct nanohub_data *data = chub->data;

	nanohub_dev_info(chub->dev,
			 "%s: nanohub driver status\nwu:%d wu_l:%d acq:%d irq1_apInt:%d fired:%d\n",
			 __func__, atomic_read(&data->wakeup_cnt),
			 atomic_read(&data->wakeup_lock_cnt),
			 atomic_read(&data->wakeup_acquired),
			 atomic_read(&chub->irq1_apInt), nanohub_irq1_fired(data));
	print_chub_user(data);
#endif

	nanohub_dev_info(chub->dev, "%s: status:%d, reset_cnt:%d, wakeup_by_chub_cnt:%d\n",
			 __func__, atomic_read(&chub->chub_status),
			 chub->err_cnt[CHUB_ERR_RESET_CNT], chub->wakeup_by_chub_cnt);
	/* print error status */
	for (i = 0; i < CHUB_ERR_MAX; i++) {
		if (chub->err_cnt[i])
			nanohub_dev_info(chub->dev, "%s: err(%d) : err_cnt:%d times\n",
					 __func__, i, chub->err_cnt[i]);
	}
#ifdef USE_FW_DUMP
	contexthub_ipc_write_event(chub, MAILBOX_EVT_DUMP_STATUS);
#endif
	ipc_dump();
}

void chub_dbg_print_funcname(struct contexthub_ipc_info *chub, u32 lr)
{
	u32 index;
	u32 offset;
	u32 i;
	char *funcname_table;
	char funcname[50];

/*
	nanohub_dev_info(ipc->dev, "%s - symbol_table:%llx\n", __func__, ipc->symbol_table);
	nanohub_dev_info(ipc->dev, "%s - symbol_table->size:%x\n", __func__, ipc->symbol_table->size);
	nanohub_dev_info(ipc->dev, "%s - symbol_table->count:%x\n", __func__, ipc->symbol_table->count);
	nanohub_dev_info(ipc->dev, "%s - symbol_table->name_offset:%x\n", __func__, ipc->symbol_table->name_offset);
	nanohub_dev_info(ipc->dev, "%s - symbol_table->symbol:%llx\n", __func__, ipc->symbol_table->symbol);

	print_hex_dump(KERN_CONT, "nanohub :",
                     DUMP_PREFIX_OFFSET, 16, 1, ipc->symbol_table, 32, false);
*/

	funcname_table = (char *)(chub->symbol_table) + chub->symbol_table->size;

	for (index = 0 ; index < chub->symbol_table->count ; index++) {
		if ((chub->symbol_table->symbol[index].base <= lr) &&
		    ((lr - chub->symbol_table->symbol[index].base) <
		    chub->symbol_table->symbol[index].size))
			break;
	}

	if (index >= chub->symbol_table->count)
		return;

	offset = lr - chub->symbol_table->symbol[index].base;

	for (i = 0 ; i < chub->symbol_table->symbol[index].length ; i++)
		funcname[i] = funcname_table[chub->symbol_table->symbol[index].offset + i];
	funcname[i] = '\0';

	nanohub_dev_info(chub->dev, "[ %08x ] %s + 0x%x\n", lr, funcname, offset);
}

void chub_dbg_call_trace(struct contexthub_ipc_info *chub)
{
	struct RamPersistedDataAndDropbox *dbx;
	void __iomem *sp, *stack_top;
	u32 pc, lr;
	u32 code_start, code_end;
	u32 count = 0;
	u16 opcode1, opcode2;

	nanohub_dev_info(chub->dev, "%s - Dump CHUB call stack\n", __func__);
	if (!chub->symbol_table) {
		nanohub_dev_info(chub->dev, "%s - there is no symbol_table\n", __func__);
		return;
	}

	dbx = (struct RamPersistedDataAndDropbox *)ipc_get_base(IPC_REG_PERSISTBUF);

	if (dbx->magic == 0x31416200) {
		sp = ipc_get_base(IPC_REG_BL) + dbx->r[13];
		pc = dbx->r[15];
		lr = dbx->r[14];
		nanohub_dev_info(chub->dev,
				 "%s : Get PC/LR from Dropbox : %llx %llx\n", __func__, pc, lr);
	} else {
		sp = ipc_get_base(IPC_REG_BL) + p_dbg_dump->gpr[13];
		pc = p_dbg_dump->gpr[16];
		lr = p_dbg_dump->gpr[14];
		nanohub_dev_info(chub->dev,
				 "%s : Get PC/LR from GPR : %llx %llx\n", __func__, pc, lr);
	}

	stack_top = ipc_get_base(IPC_REG_BL) + (u32)__raw_readl(ipc_get_base(IPC_REG_OS));

	if (sp >= stack_top)
		return;

	chub_dbg_print_funcname(chub, pc);

	code_start = (uintptr_t)ipc_get_base(IPC_REG_OS) - (uintptr_t)ipc_get_base(IPC_REG_BL);
	code_end = code_start + (u32)ipc_get_size(IPC_REG_OS);

	while (sp < stack_top && count < 4) {
		lr = (u32)__raw_readl(sp);

		sp += 4;

		if ((lr & 0x1) == 0) {
			continue;
		}

		lr = lr - 1;
		if (lr <= code_start + 0x100 || lr > code_end) {
			continue;
		}

		opcode1 = (u16)__raw_readw((ipc_get_base(IPC_REG_BL) + lr - 2));
		opcode2 = (u16)__raw_readw((ipc_get_base(IPC_REG_BL) + lr - 4));

		if ((opcode1 & 0xF800) == 0x4000) {
			lr = lr - 2;
		} else if ((opcode2 & 0xF000) == 0xF000) {
			lr = lr - 4;
		} else {
			continue;
		}

		chub_dbg_print_funcname(chub, lr);
		count++;
	}
	nanohub_dev_info(chub->dev, "%s : SP : %llx\n", __func__, sp);
}

void chub_dbg_dump_hw(struct contexthub_ipc_info *chub, enum chub_err_type reason)
{
	u64 ktime_now = ktime_get_boottime_ns();
	static u64 last_dump_time;

	nanohub_dev_info(chub->dev, "%s: reason:%d\n", __func__, reason);

	chub_dbg_dump_gpr(chub);

	//if (!last_dump_time || ktime_now - last_dump_time > 60ULL*1000*1000*1000)
		chub_dbg_dump_ram(reason);
	    chub_dbg_get_hardFaultInfo(chub);
	//else
	//	nanohub_dev_info(chub->dev, "%s: repeated dump in 1 min, skip sram\n", __func__);
	//last_dump_time = ktime_now;
	
	nanohub_dev_info(chub->dev, "%s : hardFault info ===========\n", __func__);
	nanohub_dev_info(chub->dev, "msp : 0x%X\n", hardFaultInfo.msp);
	nanohub_dev_info(chub->dev, "psp : 0x%X\n", hardFaultInfo.psp);
	nanohub_dev_info(chub->dev, "sp  : 0x%X\n", hardFaultInfo.sp);
	nanohub_dev_info(chub->dev, "NVIC_ISER : 0x%X\n", hardFaultInfo.NVIC_ISER);
	nanohub_dev_info(chub->dev, "NVIC_ICER : 0x%X\n", hardFaultInfo.NVIC_ICER);
	nanohub_dev_info(chub->dev, "%s : hardFault info End===========\n", __func__);

	chub_dbg_call_trace(chub);
	chub_dbg_dump_cmu(chub);
	chub_dbg_dump_sys(chub);
	chub_dbg_dump_wdt(chub);
	chub_dbg_dump_timer(chub);
	chub_dbg_dump_pwm(chub);
	chub_dbg_dump_rtc(chub);
	chub_dbg_dump_usi(chub);

#ifdef CONFIG_SENSOR_DRV
	nanohub_dev_info(chub->dev, "%s: notice to dump chub registers\n", __func__);
	nanohub_add_dump_request(chub->data);
#endif

#ifdef CONFIG_SENSORS_SSP
	//if (p_dbg_dump && (!last_dump_time || ktime_now - last_dump_time > 60ULL*1000*1000*1000)) {
		ssp_dump_write_file(chub->ssp_data, p_dbg_dump->time / NSEC_PER_SEC, reason,
			p_dbg_dump->sram, ipc_get_chub_mem_size(), p_dbg_dump->gpr, NUM_OF_GPR, (u32 *)&hardFaultInfo);
	//} else {
	//	nanohub_dev_info(chub->dev, "%s: repeated dump in 1 min, skip ssp dump\n", __func__);
	//}
#endif
	last_dump_time = ktime_now;
}

static ssize_t chub_get_chub_register_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct contexthub_ipc_info *chub = get_contexthub_info_from_dev(dev);
	char *pbuf = buf;
	int i;
	u32 usi_protocol;
	int j;
	int index = 0;

	if (p_dbg_dump) {
		chub_dbg_dump_cmu(chub);
		pbuf += sprintf(pbuf, "===================\n");
		pbuf += sprintf(pbuf, "CHUB CMU register\n");

		for (i = 0; i <= CMU_REG_MAX - 1; i++)
			pbuf +=
			    sprintf(pbuf, "R%02d : %08x\n", i,
				    p_dbg_dump->cmu[i]);

		chub_dbg_dump_sys(chub);
		pbuf += sprintf(pbuf, "===================\n");
		pbuf += sprintf(pbuf, "CHUB SYS register\n");

		for (i = 0; i <= SYS_REG_MAX - 1; i++)
			pbuf +=
			    sprintf(pbuf, "R%02d : %08x\n", i,
				    p_dbg_dump->sys[i]);

		chub_dbg_dump_wdt(chub);
		pbuf += sprintf(pbuf, "===================\n");
		pbuf += sprintf(pbuf, "CHUB WDT register\n");

		for (i = 0; i <= WDT_REG_MAX - 1; i++)
			pbuf +=
			    sprintf(pbuf, "R%02d : %08x\n", i,
				    p_dbg_dump->wdt[i]);

		chub_dbg_dump_timer(chub);
		pbuf += sprintf(pbuf, "===================\n");
		pbuf += sprintf(pbuf, "CHUB TIMER register\n");

		for (i = 0; i <= TIMER_REG_MAX - 1; i++)
			pbuf +=
			    sprintf(pbuf, "R%02d : %08x\n", i,
				    p_dbg_dump->timer[i]);

		chub_dbg_dump_pwm(chub);
		pbuf += sprintf(pbuf, "====================\n");
		pbuf += sprintf(pbuf, "CHUB PWM register\n");

		for (i = 0; i <= PWM_REG_MAX - 1; i++)
			pbuf +=
			    sprintf(pbuf, "R%02d : %08x\n", i,
				    p_dbg_dump->pwm[i]);

		chub_dbg_dump_rtc(chub);
		pbuf +=	sprintf(pbuf, "===================\n");
		pbuf += sprintf(pbuf, "CHUB RTC register\n");

		for (i = 0; i <= RTC_REG_MAX - 1; i++)
			pbuf +=
			    sprintf(pbuf, "R%02d : %08x\n", i,
				    p_dbg_dump->rtc[i]);

		chub_dbg_dump_usi(chub);

		for (j = 0; j < chub->usi_cnt; j++) {
			usi_protocol = READ_CHUB_USI_CONF(chub->usi_array[j]);
			switch (usi_protocol) {
			case USI_PROTOCOL_UART:
				pbuf +=
				    sprintf(pbuf, "===================\n");
				pbuf +=
				    sprintf(pbuf,
					    "CHUB USI%d UART register\n", j);

				for (i = 0; i <= UART_REG_MAX - 1; i++)
					pbuf +=
					    sprintf(pbuf,
						    "R%02d : %08x\n", i,
						    p_dbg_dump->usi[index++]);
				break;
			case USI_PROTOCOL_SPI:
				pbuf +=
				    sprintf(pbuf, "===================\n");
				pbuf +=
				    sprintf(pbuf,
					    "CHUB USI%d SPI register\n", j);

				for (i = 0; i <= SPI_REG_MAX - 1; i++)
					pbuf +=
					    sprintf(pbuf,
						    "R%02d : %08x\n", i,
						    p_dbg_dump->usi[index++]);
				break;
			case USI_PROTOCOL_I2C:
				pbuf +=
				    sprintf(pbuf, "===================\n");
				pbuf +=
				    sprintf(pbuf,
					    "CHUB USI%d I2C register\n", j);

				for (i = 0; i <= I2C_REG_MAX - 1; i++)
					pbuf +=
					    sprintf(pbuf,
						    "R%02d : %08x\n", i,
						    p_dbg_dump->usi[index++]);
				break;
			default:
				break;
			}
		}
	}

	if ((u32)(pbuf - buf) > 4096)
		nanohub_dev_err(dev, "show size (%u) bigger than 4096\n",
			(u32)(pbuf - buf));

	return pbuf - buf;
}

static ssize_t chub_get_gpr_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct contexthub_ipc_info *chub = get_contexthub_info_from_dev(dev);
	char *pbuf = buf;
	int i;

	if (p_dbg_dump) {
		chub_dbg_dump_gpr(chub);

		pbuf +=
		    sprintf(pbuf, "========================================\n");
		pbuf += sprintf(pbuf, "CHUB CPU register dump\n");

		for (i = 0; i <= 15; i++)
			pbuf +=
			    sprintf(pbuf, "R%02d        : %08x\n", i,
				    p_dbg_dump->gpr[i]);

		pbuf +=
		    sprintf(pbuf, "PC         : %08x\n",
			    p_dbg_dump->gpr[GPR_PC_INDEX]);
		pbuf +=
		    sprintf(pbuf, "========================================\n");

#if IS_ENABLED(CONFIG_EXYNOS_LPD)
		pbuf += sprintf(pbuf, "CM55 register dump\n");
		pbuf += sprintf(pbuf, "INST_CUR   : 0x%x\n",
				p_dbg_dump->cm55_cur);
		pbuf += sprintf(pbuf, "INST_CTRL  : 0x%x\n",
				p_dbg_dump->cm55_ctrl);
		pbuf += sprintf(pbuf, "INST_FAULT : 0x%x 0x%x\n",
				p_dbg_dump->cm55_fault[0], p_dbg_dump->cm55_fault[1]);
		pbuf += sprintf(pbuf, "USER       : 0x%x 0x%x 0x%x 0x%x 0x%x\n",
				p_dbg_dump->cm55_user[0], p_dbg_dump->cm55_user[1],
				p_dbg_dump->cm55_user[2], p_dbg_dump->cm55_user[3],
				p_dbg_dump->cm55_user[4]);
		pbuf +=
		    sprintf(pbuf, "========================================\n");
#endif
	}

	return pbuf - buf;
}

static ssize_t chub_bin_dump_registers_read(struct file *file,
					    struct kobject *kobj,
					    struct bin_attribute *battr,
					    char *buf, loff_t off, size_t size)
{
	memcpy(buf, battr->private + off, size);
	return size;
}

static ssize_t chub_bin_sram_read(struct file *file, struct kobject *kobj,
				  struct bin_attribute *battr, char *buf,
				  loff_t off, size_t size)
{
	memcpy_fromio(buf, battr->private + off, size);
	return size;
}

static ssize_t chub_bin_dram_read(struct file *file, struct kobject *kobj,
				  struct bin_attribute *battr, char *buf,
				  loff_t off, size_t size)
{
	memcpy(buf, battr->private + off, size);
	return size;
}

static ssize_t chub_bin_dumped_sram_read(struct file *file, struct kobject *kobj,
				  struct bin_attribute *battr, char *buf,
				  loff_t off, size_t size)
{
	memcpy(buf, battr->private + off, size);
	return size;
}

static ssize_t chub_bin_logbuf_dram_read(struct file *file, struct kobject *kobj,
				  struct bin_attribute *battr, char *buf,
				  loff_t off, size_t size)
{
	memcpy(buf, battr->private + off, size);
	return size;
}

static ssize_t chub_bin_dfs_read(struct file *file, struct kobject *kobj,
				  struct bin_attribute *battr, char *buf,
				  loff_t off, size_t size)
{
	memcpy_fromio(buf, battr->private + off, size);
	return size;
}

static BIN_ATTR_RO(chub_bin_dram, 0);
static BIN_ATTR_RO(chub_bin_dumped_sram, 0);
static BIN_ATTR_RO(chub_bin_logbuf_dram, 0);
static BIN_ATTR_RO(chub_bin_dfs, 0);
static BIN_ATTR_RO(chub_bin_sram, 0);
static BIN_ATTR_RO(chub_bin_dump_registers, 0);

static struct bin_attribute *chub_bin_attrs[] = {
	&bin_attr_chub_bin_sram,
	&bin_attr_chub_bin_dram,
	&bin_attr_chub_bin_dumped_sram,
	&bin_attr_chub_bin_logbuf_dram,
	&bin_attr_chub_bin_dfs,
	&bin_attr_chub_bin_dump_registers,
};

#define SIZE_UTC_NAME (32)

#define IPC_DBG_UTC_CIPC_TEST (IPC_DEBUG_UTC_REBOOT + 1)
char chub_utc_name[][SIZE_UTC_NAME] = {
	[IPC_DEBUG_UTC_STOP] = "stop",
	[IPC_DEBUG_UTC_AGING] = "aging",
	[IPC_DEBUG_UTC_WDT] = "wdt",
	[IPC_DEBUG_UTC_IDLE] = "idle",
	[IPC_DEBUG_UTC_TIMER] = "timer",
	[IPC_DEBUG_UTC_MEM] = "mem",
	[IPC_DEBUG_UTC_GPIO] = "gpio",
	[IPC_DEBUG_UTC_SPI] = "spi",
	[IPC_DEBUG_UTC_CMU] = "cmu",
	[IPC_DEBUG_UTC_GPIO] = "gpio",
	[IPC_DEBUG_UTC_TIME_SYNC] = "time_sync",
	[IPC_DEBUG_UTC_ASSERT] = "assert",
	[IPC_DEBUG_UTC_FAULT] = "fault",
	[IPC_DEBUG_UTC_CHECK_STATUS] = "stack",
	[IPC_DEBUG_UTC_CHECK_CPU_UTIL] = "utilization",
	[IPC_DEBUG_UTC_HEAP_DEBUG] = "heap",
	[IPC_DEBUG_UTC_HANG] = "hang",
	[IPC_DEBUG_UTC_HANG_ITMON] = "itmon",
	[IPC_DEBUG_UTC_DFS] = "dfs",
	[IPC_DEBUG_UTC_HANG_IPC_C2A_FULL] = "ipc_c2a_evt_full",
	[IPC_DEBUG_UTC_HANG_IPC_C2A_CRASH] = "ipc_c2a_evt_crash",
	[IPC_DEBUG_UTC_HANG_IPC_C2A_DATA_FULL] = "ipc_c2a_data_full",
	[IPC_DEBUG_UTC_HANG_IPC_C2A_DATA_CRASH] = "ipc_c2a_data_crash",
	[IPC_DEBUG_UTC_HANG_IPC_A2C_FULL] = "ipc_a2c_evt_full",
	[IPC_DEBUG_UTC_HANG_IPC_A2C_CRASH] = "ipc_a2c_evt_crash",
	[IPC_DEBUG_UTC_HANG_IPC_A2C_DATA_FULL] = "ipc_a2c_data_full",
	[IPC_DEBUG_UTC_HANG_IPC_A2C_DATA_CRASH] = "ipc_a2c_data_crash",
	[IPC_DEBUG_UTC_HANG_IPC_LOGBUF_EQ_CRASH] = "ipc_logbuf_eq_crash",
	[IPC_DEBUG_UTC_HANG_IPC_LOGBUF_DQ_CRASH] = "ipc_logbuf_dq_crash",
	[IPC_DEBUG_UTC_HANG_INVAL_INT] = "ipc_inval_int",
	[IPC_DEBUG_UTC_REBOOT] = "reboot(CSP_REBOOT)",
	[IPC_DBG_UTC_CIPC_TEST] = "cipc debug", /* ap can handle it */
};

#define SIZE_DFS_NAME   (32)
char chub_dfs_name[][SIZE_DFS_NAME] = {
	[DFS_GOVERNOR_OFF] = "dfs_off",
	[DFS_GOVERNOR_SIMPLE] = "dfs_simple_governor",
	[DFS_GOVERNOR_POWER] = "dfs_power_governor",
};

static ssize_t chub_alive_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int index = 0;
	struct contexthub_ipc_info *chub = get_contexthub_info_from_dev(dev);
	int ret = contexthub_ipc_write_event(chub, MAILBOX_EVT_CHUB_ALIVE);

	if (!ret)
		index += sprintf(buf, "chub alive\n");
	else
		index += sprintf(buf, "chub isn't alive\n");

	return index;
}

static ssize_t chub_cpuinfo_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
        char *pbuf = buf;
        struct ipc_map_area *map = ipc_get_base(IPC_REG_IPC);

        pbuf +=
                snprintf(pbuf, PAGE_SIZE, "pdcnt  : %d\n", map->cpu_info.pgcnt);
        pbuf +=
                snprintf(pbuf, PAGE_SIZE, "pdtime : %lld\n", map->cpu_info.pgtime);
        pbuf +=
                snprintf(pbuf, PAGE_SIZE, "cdcnt  : %d\n", map->cpu_info.cgcnt);
        pbuf +=
                snprintf(pbuf, PAGE_SIZE, "cdtime : %lld\n", map->cpu_info.cgtime);
        pbuf +=
                snprintf(pbuf, PAGE_SIZE, "curtime: %lld\n", map->cpu_info.curtime);
        return pbuf - buf;
}

#if IS_ENABLED(CONFIG_SEC_PM_DEBUG)
void chub_dbg_cpuinfo(int ap_status)
{
	struct ipc_map_area *map = ipc_get_base(IPC_REG_IPC);

	if (!map) {
		pr_err("%s: can't get ipc base for chub\n", __func__);
		return;
	}

	pr_info("%s: pg(%d, %lld) cg(%d, %lld) time(%lld) %s\n", __func__,
			map->cpu_info.pgcnt,
			map->cpu_info.pgtime,
			map->cpu_info.cgcnt,
			map->cpu_info.cgtime,
			map->cpu_info.curtime,
			ap_status == 1 ? "AP_WAKE" : "AP_SLEEP");
	pr_info("%s: pg_diff(%d, %lld) cg_diff(%d, %lld) time_diff(%lld) %s\n", __func__,
			map->cpu_info.pgcnt - sec_pm_chub_cpu.pgcnt,
			map->cpu_info.pgtime - sec_pm_chub_cpu.pgtime,
			map->cpu_info.cgcnt - sec_pm_chub_cpu.cgcnt,
			map->cpu_info.cgtime - sec_pm_chub_cpu.cgtime,
			map->cpu_info.curtime - sec_pm_chub_cpu.curtime,
			ap_status == 1 ? "AP_WAKE" : "AP_SLEEP");

	memcpy(&sec_pm_chub_cpu, &map->cpu_info, sizeof(struct ipc_cpu_info));
}
#endif

static ssize_t chub_utc_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int i;
	int index = 0;

	for (i = 0; i < sizeof(chub_utc_name) / SIZE_UTC_NAME; i++)
		if (chub_utc_name[i][0])
			index +=
			    sprintf(buf + index, "%d %s\n", i,
				    chub_utc_name[i]);

	return index;
}

static ssize_t chub_utc_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct contexthub_ipc_info *chub = get_contexthub_info_from_dev(dev);
	long event;
	int err;

	err = kstrtol(&buf[0], 10, &event);
	nanohub_dev_info(chub->dev, "%s: event:%d\n", __func__, event);

	if (!err) {
		contexthub_ipc_write_event(chub, event);

		if (event >= IPC_DEBUG_UTC_HANG_IPC_C2A_FULL)
			ipc_dump();
		return count;
	} else {
		return 0;
	}
}

static ssize_t chub_dfs_gov_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int i;
	int index = 0;

	for (i = 0; i < sizeof(chub_dfs_name) / SIZE_DFS_NAME; i++)
		if (chub_dfs_name[i][0])
			index +=
			    sprintf(buf + index, "%d %s\n", i,
				    chub_dfs_name[i]);

	return index;
}

static ssize_t chub_dfs_gov_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct contexthub_ipc_info *chub = get_contexthub_info_from_dev(dev);
	long event;
	int ret;

	ret = kstrtol(&buf[0], 10, &event);
	if (ret)
		return ret;

	ipc_write_value(IPC_VAL_A2C_DFS, event);
	nanohub_dev_info(chub->dev, "%s: event: %d, %d\n",
			 __func__, event, ipc_read_value(IPC_VAL_A2C_DFS));
	contexthub_ipc_write_event(chub, MAILBOX_EVT_DFS_GOVERNOR);

	return ret ? ret : count;
}

struct chub_ipc_utc {
	char name[IPC_NAME_MAX];
	enum cipc_region reg;
};

static struct chub_ipc_utc ipc_utc[] = {
	{"AP2CHUB", CIPC_REG_DATA_CHUB2AP},
	{"AP2CHUB_BATCH", CIPC_REG_DATA_CHUB2AP_BATCH},
	{"ABOX2CHUB", CIPC_REG_DATA_CHUB2ABOX},
	{"ABOX2CHUB_BAAW", CIPC_REG_DATA_CHUB2ABOX | (1 << CIPC_TEST_BAAW_REQ_BIT)},
	{"CIPC_RESET", 0},
};

static ssize_t chub_ipc_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int i;
	int index = 0;

	for (i = 0; i < sizeof(ipc_utc) / sizeof(struct chub_ipc_utc); i++)
		index +=
		    sprintf(buf + index, "%d %s\n", i, ipc_utc[i].name);

	return index;
}

#define CIPC_TEST_SIZE (64)
static ssize_t chub_ipc_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct contexthub_ipc_info *chub = get_contexthub_info_from_dev(dev);
	char input[CIPC_TEST_SIZE];
	char output[CIPC_TEST_SIZE];
	int ret;
	long event;
	int err;
	int i;

	err = kstrtol(&buf[0], 10, &event);
	nanohub_dev_info(chub->dev, "%s: event:%d\n", __func__, event);

	if (event >= ARRAY_SIZE(ipc_utc)) {
		return -EINVAL;
	}

	if (!ipc_utc[event].reg) {
		nanohub_dev_info(chub->dev, "%s: cipc reset\n", __func__);
		cipc_reset_map();
		return count;
	}

	memset(input, 0, CIPC_TEST_SIZE);
	memset(output, 0, CIPC_TEST_SIZE);

	if (count <= CIPC_TEST_SIZE) {
		memset(output, 0, CIPC_TEST_SIZE);
		for (i = 0; i < CIPC_TEST_SIZE; i++)
			output[i] = i;
	} else {
		nanohub_dev_err(chub->dev, "%s: ipc size(%d) is bigger than max(%d)\n",
				__func__, (int)count, (int)CIPC_TEST_SIZE);
		return -EINVAL;
	}

	nanohub_dev_err(chub->dev, "%s: event:%d, reg:%d\n", __func__, event, ipc_utc[event].reg);

	ipc_write_value(IPC_VAL_A2C_DEBUG2, ipc_utc[event].reg);

	ret = contexthub_ipc_write_event(chub, (u32)IPC_DEBUG_UTC_IPC_TEST_START);
	if (ret) {
		nanohub_dev_err(chub->dev,
				"%s: fails to set start test event. ret:%d\n", __func__, ret);
		count = ret;
		goto out;
	}

	if (event == IPC_DEBUG_UTC_IPC_AP) {
		ret = contexthub_ipc_write(chub, input, count, IPC_MAX_TIMEOUT);
		if (ret != count) {
			nanohub_dev_info(chub->dev, "%s: fail to write\n", __func__);
			goto out;
		}

		ret = contexthub_ipc_read(chub, output, 0, IPC_MAX_TIMEOUT);
		if (count != ret) {
			nanohub_dev_info(chub->dev, "%s: fail to read ret:%d\n", __func__, ret);
		}

		if (strncmp(input, output, count)) {
			nanohub_dev_info(chub->dev, "%s: fail to compare input/output\n", __func__);
			print_hex_dump(KERN_CONT, "chub input:",
				       DUMP_PREFIX_OFFSET, 16, 1, input,
				       count, false);
			print_hex_dump(KERN_CONT, "chub output:",
				       DUMP_PREFIX_OFFSET, 16, 1, output,
				       count, false);
		} else
			nanohub_dev_info(chub->dev,
					 "[%s pass] len:%d, str: %s\n", __func__,
					 (int)count, output);
	} else {
		nanohub_dev_err(chub->dev, "%s: %d: %s. reg:%d\n",
				__func__, event, ipc_utc[event].name, ipc_utc[event].reg);
		msleep(1000); /* wait util chub recived it */
	}

	out:
		ret = contexthub_ipc_write_event(chub, (u32)IPC_DEBUG_UTC_IPC_TEST_END);
		if (ret) {
			nanohub_dev_err(chub->dev, "%s: fails to set end test event. ret:%d\n",
					__func__, ret);
			count = ret;
		}
	return count;
}

static ssize_t chub_get_dump_status_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct contexthub_ipc_info *chub = get_contexthub_info_from_dev(dev);

	chub_dbg_dump_status(chub);
	return count;
}

static ssize_t chub_set_dump_hw_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct contexthub_ipc_info *chub = get_contexthub_info_from_dev(dev);

	chub_dbg_dump_hw(chub, 0);
	return count;
}

static ssize_t chub_loglevel_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct contexthub_ipc_info *chub = get_contexthub_info_from_dev(dev);
	enum ipc_fw_loglevel loglevel = chub->chub_rt_log.loglevel;
	int index = 0;

	nanohub_dev_info(dev, "%s: %d\n", __func__, loglevel);
	index += sprintf(buf, "%d:%s, %d:%s, %d:%s\n",
		CHUB_RT_LOG_OFF, "off", CHUB_RT_LOG_DUMP, "dump-only",
		CHUB_RT_LOG_DUMP_PRT, "dump-prt");
	index += sprintf(buf + index, "cur-loglevel: %d: %s\n",
		loglevel, !loglevel ? "off" : ((loglevel == CHUB_RT_LOG_DUMP) ? "dump-only" : "dump-prt"));

	return index;
}

static ssize_t chub_loglevel_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct contexthub_ipc_info *chub = get_contexthub_info_from_dev(dev);
	long event;
	int ret;

	ret = kstrtol(&buf[0], 10, &event);
	if (ret)
		return ret;

	chub->chub_rt_log.loglevel = (enum ipc_fw_loglevel)event;
	nanohub_dev_info(dev, "%s: %d->%d\n", __func__, event, chub->chub_rt_log.loglevel);
	contexthub_ipc_write_event(chub, MAILBOX_EVT_RT_LOGLEVEL);

	return ret ? ret : count;
}

#if IS_ENABLED(CONFIG_EXYNOS_LPD)
static ssize_t chub_lpd_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int index = 0;

	index += sprintf(buf + index, "1: reset by chub\n");
	index += sprintf(buf + index, "2: reset by chub with image download\n");
	index += sprintf(buf + index, "3: reset by AP with image download\n");

	return index;
}

static ssize_t chub_lpd_start(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct contexthub_ipc_info *chub = get_contexthub_info_from_dev(dev);
	long num;
	int ret;

	ret = kstrtol(&buf[0], 10, &num);
	if (ret)
		return ret;
	switch (num) {
		case LPD_STOP:
			contexthub_ipc_write_event(chub, MAILBOX_EVT_STOP_LPD);
			break;
		case LPD_START:
			contexthub_ipc_write_event(chub, MAILBOX_EVT_START_LPD);
			break;
		case LPD_DL_START:
			ret = contexthub_download_cm55_image(chub);
			if (ret) {
				nanohub_dev_err(dev, "%s: cm55 download fail!\n", __func__);
				return -1;
			}
			contexthub_ipc_write_event(chub, MAILBOX_EVT_START_LPD);
			break;
		case LPD_DL_START_AP:
			ret = contexthub_download_cm55_image(chub);
			if (ret) {
				nanohub_dev_err(dev, "%s: cm55 download fail!\n", __func__);
				return -1;
			}
			if (chub->sysreg_chub) {
				__raw_writel(0x2000, chub->sysreg_chub + 0x1000);
				__raw_writel(0x2000, chub->sysreg_chub + 0x1004);
				ret = cal_chub_cm55_reset_release();
			} else {
				nanohub_dev_err(dev, "%s: sysreg for cm55 not found!\n", __func__);
				return -1;
			}
			break;
		default:
			nanohub_info("%s: not defined %d\n", __func__, num);
			break;
	}

	return ret ? ret : count;
}
#endif

static struct device_attribute attributes[] = {
	__ATTR(get_gpr, 0440, chub_get_gpr_show, NULL),
	__ATTR(get_chub_register, 0440, chub_get_chub_register_show, NULL),
	__ATTR(dump_status, 0220, NULL, chub_get_dump_status_store),
	__ATTR(dump_hw, 0220, NULL, chub_set_dump_hw_store),
	__ATTR(utc, 0664, chub_utc_show, chub_utc_store),
	__ATTR(dfs_gov, 0664, chub_dfs_gov_show, chub_dfs_gov_store),
	__ATTR(ipc_test, 0664, chub_ipc_show, chub_ipc_store),
	__ATTR(alive, 0440, chub_alive_show, NULL),
	__ATTR(cpuinfo, 0440, chub_cpuinfo_show, NULL),
	__ATTR(loglevel, 0664, chub_loglevel_show, chub_loglevel_store),
#if IS_ENABLED(CONFIG_EXYNOS_LPD)
	__ATTR(lpd, 0664, chub_lpd_show, chub_lpd_start),
#endif
};

void *chub_dbg_get_memory(struct device_node *node)
{
	struct device_node *np;

	pr_info("%s: chub_rmem\n", __func__);

	np = of_parse_phandle(node, "memory-region", 0);
	if (!np)
		pr_err("%s memory region not parsed!!");
	else
		chub_rmem = of_reserved_mem_lookup(np);

	if (!chub_rmem) {
		pr_err("%s: rmem not available, kmalloc instead", __func__);
		p_dbg_dump = kmalloc(SZ_2M, GFP_KERNEL);
	} else
		p_dbg_dump = phys_to_virt(chub_rmem->base);

	memset(p_dbg_dump, 0, SRAM_ALIGN);

	return &p_dbg_dump->chub;
}

int chub_dbg_init(struct contexthub_ipc_info *chub, void *kernel_logbuf, int kernel_logbuf_size)
{
	int i, ret = 0;
	enum dbg_dump_area area;

	struct device *dev;
	struct device *sensor_dev = NULL;

	if (!chub)
		return -EINVAL;

	sensor_dev = dev = chub->dev;
#ifdef CONFIG_SENSOR_DRV
	if (chub->data)
		sensor_dev = chub->data->io[ID_NANOHUB_SENSOR].dev;
#endif

	nanohub_info("%s: %s: %s\n", __func__, dev_name(dev), dev_name(sensor_dev));

	bin_attr_chub_bin_dumped_sram.size = ipc_get_chub_mem_size();
	bin_attr_chub_bin_dumped_sram.private = p_dbg_dump->sram;

	bin_attr_chub_bin_dram.size = sizeof(struct dbg_dump);
	bin_attr_chub_bin_dram.private= p_dbg_dump;

	bin_attr_chub_bin_sram.size = ipc_get_chub_mem_size();
	bin_attr_chub_bin_sram.private = ipc_get_base(IPC_REG_DUMP);

	bin_attr_chub_bin_logbuf_dram.size = kernel_logbuf_size;
	bin_attr_chub_bin_logbuf_dram.private = kernel_logbuf;

	bin_attr_chub_bin_dfs.size = sizeof(struct chub_dfs);
	bin_attr_chub_bin_dfs.private = ipc_get_base(IPC_REG_IPC) + CHUB_PERSISTBUF_SIZE;
	bin_attr_chub_bin_dump_registers.size = get_chub_dumped_registers(chub->usi_cnt);
	bin_attr_chub_bin_dump_registers.private = p_dbg_dump->gpr;

	if (chub_rmem && chub_rmem->size < get_dbg_dump_size())
		nanohub_dev_err(dev,
			"rmem size (%u) should be bigger than dump size(%u)\n",
			(u32)chub_rmem->size, get_dbg_dump_size());

	for (i = 0; i < ARRAY_SIZE(chub_bin_attrs); i++) {
		struct bin_attribute *battr = chub_bin_attrs[i];
		battr->attr.mode = 0440;

		ret = device_create_bin_file(sensor_dev, battr);
		if (ret < 0)
			nanohub_dev_warn(sensor_dev, "Failed to create file: %s\n",
				 battr->attr.name);
	}

	for (i = 0, ret = 0; i < ARRAY_SIZE(attributes); i++) {
		ret = device_create_file(sensor_dev, &attributes[i]);
		if (ret)
			nanohub_dev_warn(dev, "Failed to create file: %s\n",
				 attributes[i].attr.name);
	}

	area = DBG_IPC_AREA;
	strncpy(p_dbg_dump->info[area].name, "ipc_map", AREA_NAME_MAX);
	p_dbg_dump->info[area].offset =
	    (void *)p_dbg_dump->ipc_addr - (void *)p_dbg_dump;
	p_dbg_dump->info[area].size = sizeof(struct ipc_area) * IPC_REG_MAX;

	area = DBG_NANOHUB_DD_AREA;
	strncpy(p_dbg_dump->info[area].name, "nano_dd", AREA_NAME_MAX);
	p_dbg_dump->info[area].offset =
	    (void *)&p_dbg_dump->chub - (void *)p_dbg_dump;
	p_dbg_dump->info[area].size = sizeof(struct contexthub_ipc_info);

	area = DBG_GPR_AREA;
	strncpy(p_dbg_dump->info[area].name, "gpr", AREA_NAME_MAX);
	p_dbg_dump->info[area].offset =
	    (void *)p_dbg_dump->gpr - (void *)p_dbg_dump;
	p_dbg_dump->info[area].size = sizeof(u32) * NUM_OF_GPR;

	area = DBG_CMU_AREA;
	strncpy(p_dbg_dump->info[area].name, "cmu", AREA_NAME_MAX);
	p_dbg_dump->info[area].offset =
		(void *)p_dbg_dump->cmu - (void *)p_dbg_dump;
	p_dbg_dump->info[area].size = sizeof(u32) * CMU_REG_MAX;

	area = DBG_SYS_AREA;
	strncpy(p_dbg_dump->info[area].name, "sys", AREA_NAME_MAX);
	p_dbg_dump->info[area].offset =
	    (void *)p_dbg_dump->sys - (void *)p_dbg_dump;
	p_dbg_dump->info[area].size = sizeof(u32) * SYS_REG_MAX;

	area = DBG_WDT_AREA;
	strncpy(p_dbg_dump->info[area].name, "wdt", AREA_NAME_MAX);
	p_dbg_dump->info[area].offset =
	    (void *)p_dbg_dump->wdt - (void *)p_dbg_dump;
	p_dbg_dump->info[area].size = sizeof(u32) * WDT_REG_MAX;

	area = DBG_TIMER_AREA;
	strncpy(p_dbg_dump->info[area].name, "timer", AREA_NAME_MAX);
	p_dbg_dump->info[area].offset =
	    (void *)p_dbg_dump->timer - (void *)p_dbg_dump;
	p_dbg_dump->info[area].size = sizeof(u32) * TIMER_REG_MAX;

	area = DBG_PWM_AREA;
	strncpy(p_dbg_dump->info[area].name, "pwm", AREA_NAME_MAX);
	p_dbg_dump->info[area].offset =
	    (void *)p_dbg_dump->pwm - (void *)p_dbg_dump;
	p_dbg_dump->info[area].size = sizeof(u32) * PWM_REG_MAX;

	area = DBG_RTC_AREA;
	strncpy(p_dbg_dump->info[area].name, "rtc", AREA_NAME_MAX);
	p_dbg_dump->info[area].offset =
	    (void *)p_dbg_dump->rtc - (void *)p_dbg_dump;
	p_dbg_dump->info[area].size = sizeof(u32) * RTC_REG_MAX;

	area = DBG_USI_AREA;
	strncpy(p_dbg_dump->info[area].name, "usi", AREA_NAME_MAX);
	p_dbg_dump->info[area].offset =
	    (void *)p_dbg_dump->usi - (void *)p_dbg_dump;
	p_dbg_dump->info[area].size = sizeof(u32) *
						USI_REG_MAX * MAX_USI_CNT;

	area = DBG_SRAM_AREA;
	strncpy(p_dbg_dump->info[area].name, "sram", AREA_NAME_MAX);
	p_dbg_dump->info[area].offset = offsetof(struct dbg_dump, sram);
	p_dbg_dump->info[area].size = bin_attr_chub_bin_sram.size;

	nanohub_dev_info(dev,
		"%s is mapped (startoffset:%d) with dump size %u\n",
		"dump buffer", offsetof(struct dbg_dump, sram), get_dbg_dump_size());

	return ret;
}
