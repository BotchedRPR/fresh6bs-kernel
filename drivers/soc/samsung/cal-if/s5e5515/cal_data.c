#include "../pmucal/include/pmucal_common.h"
#include "../pmucal/include/pmucal_cpu.h"
#include "../pmucal/include/pmucal_local.h"
#include "../pmucal/include/pmucal_rae.h"
#include "../pmucal/include/pmucal_system.h"
#include "../pmucal/include/pmucal_powermode.h"

#include "pmucal/flexpmu_cal_cpu_s5e5515.h"
#include "pmucal/flexpmu_cal_local_s5e5515.h"
#include "pmucal/flexpmu_cal_p2vmap_s5e5515.h"
#include "pmucal/flexpmu_cal_system_s5e5515.h"
#include "pmucal/flexpmu_cal_define_s5e5515.h"
#include "pmucal/flexpmu_cal_lpd_sram_s5e5515.h"


#if IS_ENABLED(CONFIG_CP_PMUCAL)
#include "../pmucal/include/pmucal_cp.h"
#include "pmucal/pmucal_cp_s5e5515.h"
#endif

#if IS_ENABLED(CONFIG_GNSS_PMUCAL)
#include "../pmucal/include/pmucal_gnss.h"
#include "pmucal/pmucal_gnss_s5e5515.h"
#endif

#include "../pmucal/include/pmucal_chub.h"
#include "./pmucal/pmucal_chub_s5e5515.h"

#include "cmucal/cmucal-node.c"
#include "cmucal/cmucal-qch.c"
#include "cmucal/cmucal-sfr.c"
#include "cmucal/cmucal-vclk.c"
#include "cmucal/cmucal-vclklut.c"

#include "cmucal/clkout_s5e5515.c"
#include "acpm_dvfs_s5e5515.h"
#include "asv_s5e5515.h"
#include "../ra.h"
#include <linux/smc.h>

#include <soc/samsung/exynos-cpupm.h>

#if IS_ENABLED(CONFIG_SEC_FACTORY)
#include <soc/samsung/exynos-pm.h>
#endif

#if defined(CONFIG_SOC_S5E5515)
#include <soc/samsung/cal-if.h>
#endif

/* defines for EWF WA */
#include <soc/samsung/cmu_ewf.h>
//#define s5e5515_CMU_BUS0_BASE	(0x1A300000)
//#define QCH_CON_TREX_D0_BUS0_QCH_OFFSET	(0x30f0)
//#define IGNORE_FORCE_PM_EN		(2)

/* defines for PLL_MMC SSC settings */
#define s5e5515_CMU_TOP_BASE		(0x15410000)

#define PLL_CON0_PLL_MMC	(0x140)
#define PLL_CON1_PLL_MMC	(0x144)
#define PLL_CON2_PLL_MMC	(0x148)
#define PLL_CON3_PLL_MMC	(0x14c)
#define PLL_CON4_PLL_MMC	(0x150)
#define PLL_CON5_PLL_MMC	(0x154)

#define PLL_ENABLE_SHIFT	(31)
#define MANUAL_MODE		(0x2)
#define PLL_MMC_MUX_BUSY_SHIFT	(16)
#define MFR_MASK		(0xff)
#define MRR_MASK		(0x3f)
#define MFR_SHIFT		(16)
#define MRR_SHIFT		(24)
#define SSCG_EN			(16)

/* defines for RCO_400 off settings */
#define s5e5515_CMU_ALIVE_BASE	(0x12800000)
#define OSC_CON0_RCO_400	(0x100)
#define OSC_CON3_RCO_400	(0x10C)
#define MUX_CLKCMU_VTS_BUS	(0x1004)
//#define MUX_CLKCMU_CMGP_ADC	(0x101c)
//#define DIV_CLKCMU_CMGP_ADC	(0x1818)
#define RCO_400_ENABLE		(31)
#define RCO_400_STABLE		(29)
#define RCO_400_MUX_SEL		(4)

/* defines for CPUCL2 smpl_warn SW release */
//#define EXYNOS9830_CCMU_CPUCL2_BASE		(0x1d200000)
//#define CCMU_SMPL_WARN_CFG		(0x9c)
//#define SW_RELEASE			(1 << 26)

void __iomem *cmu_top;
//void __iomem *cmu_bus0;
//void __iomem *ccmu_cpucl2;
void __iomem *cmu_alive;

#define NUM_SKIP_CMU_SFR	(4)
u32 skip_cmu_sfr[NUM_SKIP_CMU_SFR] = {0x1a331054, 0x1a331860, 0x1a3318fc, 0x1a33194c}; //@@@


unsigned int frac_rpll_list[10];
unsigned int frac_rpll_size;

unsigned int s5e5515_frac_rpll_list[] = {
	PLL_SHARED0,
	PLL_SHARED1,
	PLL_AUD,
};

/*
void cmu_adc_rco_400_contorl(void)
{
	unsigned int reg;

	reg = __raw_readl(cmu_alive + MUX_CLKCMU_CMGP_ADC);
	reg &= ~(1 << 0);
	__raw_writel(reg, cmu_alive + MUX_CLKCMU_CMGP_ADC);

	reg = __raw_readl(cmu_alive + DIV_CLKCMU_CMGP_ADC);
	reg &= ~(0xf << 0);
	reg |= (1 << 0);
	__raw_writel(reg, cmu_alive + DIV_CLKCMU_CMGP_ADC);
}

int cmu_vts_rco_400_control(bool on)
{
	unsigned int reg, tmp;
	unsigned int timeout = 0;

	if (on == 1) {
		reg = __raw_readl(cmu_alive + OSC_CON3_RCO_400);
		reg |= (1 << RCO_400_ENABLE);
		__raw_writel(reg, cmu_alive + OSC_CON3_RCO_400);

		while (1) {
			tmp = __raw_readl(cmu_alive + OSC_CON3_RCO_400);
			if (((tmp >> RCO_400_STABLE) & 0x1) == 1)
				break;

			timeout++;
			udelay(1);
			if (timeout > 10000) {
				pr_err("RCO_400 %s:timed out OSC_CON3_RCO_400 : 0x%x\n", __func__, tmp);

				return -ETIMEDOUT;
			}
		}

		reg = __raw_readl(cmu_alive + OSC_CON0_RCO_400);
		reg |= (1 << RCO_400_MUX_SEL);
		__raw_writel(reg, cmu_alive + OSC_CON0_RCO_400);

		reg = __raw_readl(cmu_alive + MUX_CLKCMU_VTS_BUS);
		reg |= (1 << 0);
		__raw_writel(reg, cmu_alive + MUX_CLKCMU_VTS_BUS);
	} else {
		reg = __raw_readl(cmu_alive + MUX_CLKCMU_VTS_BUS);
		reg &= ~(1 << 0);
		__raw_writel(reg, cmu_alive + MUX_CLKCMU_VTS_BUS);

		reg = __raw_readl(cmu_alive + OSC_CON0_RCO_400);
		reg &= ~(1 << RCO_400_MUX_SEL);
		__raw_writel(reg, cmu_alive + OSC_CON0_RCO_400);

		reg = __raw_readl(cmu_alive + OSC_CON3_RCO_400);
		reg &= ~(1 << RCO_400_ENABLE);
		__raw_writel(reg, cmu_alive + OSC_CON3_RCO_400);
	}

	return 0;
} EXPORT_SYMBOL(cmu_vts_rco_400_control);
*/

static int cmu_stable_done(void __iomem *cmu,
			unsigned char shift,
			unsigned int done,
			int usec)
{
	unsigned int result;

	do {
		result = get_bit(cmu, shift);

		if (result == done)
			return 0;
		udelay(1);
	} while (--usec > 0);

	return -EVCLKTIMEOUT;
}

int pll_mmc_enable(int enable)
{
	unsigned int reg;
	unsigned int cmu_mode;
	int ret;

	if (!cmu_top) {
		pr_err("%s: cmu_top cmuioremap failed\n", __func__);
		return -1;
	}

	/* set PLL to manual mode */
	cmu_mode = readl(cmu_top + PLL_CON1_PLL_MMC);
	writel(MANUAL_MODE, cmu_top + PLL_CON1_PLL_MMC);

	if (!enable) {
		/* select oscclk */
		reg = readl(cmu_top + PLL_CON0_PLL_MMC);
		reg &= ~(PLL_MUX_SEL);
		writel(reg, cmu_top + PLL_CON0_PLL_MMC);

		ret = cmu_stable_done(cmu_top + PLL_CON0_PLL_MMC, PLL_MMC_MUX_BUSY_SHIFT, 0, 100);
		if (ret)
			pr_err("pll mux change time out, \'PLL_MMC\'\n");
	}

	/* setting ENABLE of PLL */
	reg = readl(cmu_top + PLL_CON3_PLL_MMC);
	if (enable)
		reg |= 1 << PLL_ENABLE_SHIFT;
	else
		reg &= ~(1 << PLL_ENABLE_SHIFT);
	writel(reg, cmu_top + PLL_CON3_PLL_MMC);

	if (enable) {
		/* wait for PLL stable */
		ret = cmu_stable_done(cmu_top + PLL_CON3_PLL_MMC, PLL_STABLE_SHIFT, 1, 100);
		if (ret)
			pr_err("pll time out, \'PLL_MMC\' %d\n", enable);

		/* select FOUT_PLL_MMC */
		reg = readl(cmu_top + PLL_CON0_PLL_MMC);
		reg |= PLL_MUX_SEL;
		writel(reg, cmu_top + PLL_CON0_PLL_MMC);

		ret = cmu_stable_done(cmu_top + PLL_CON0_PLL_MMC, PLL_MMC_MUX_BUSY_SHIFT, 0, 100);
		if (ret)
			pr_err("pll mux change time out, \'PLL_MMC\'\n");
	}

	/* restore PLL mode */
	writel(cmu_mode, cmu_top + PLL_CON1_PLL_MMC);

	return ret;
}

int cal_pll_mmc_check(void)
{
       unsigned int reg;
       bool ret = false;

       reg = readl(cmu_top + PLL_CON4_PLL_MMC);

       if (reg & (1 << SSCG_EN))
               ret = true;

       return ret;
}
EXPORT_SYMBOL(cal_pll_mmc_check);

int cal_pll_mmc_set_ssc(unsigned int mfr, unsigned int mrr, unsigned int ssc_on)
{
	unsigned int reg;
	int ret = 0;

	/* disable PLL_MMC */
	ret = pll_mmc_enable(0);
	if (ret) {
		pr_err("%s: pll_mmc_disable failed\n", __func__);
		return ret;
	}

	/* setting MFR, MRR */
	reg = readl(cmu_top + PLL_CON5_PLL_MMC);
	reg &= ~((MFR_MASK << MFR_SHIFT) | (MRR_MASK << MRR_SHIFT));

	if (ssc_on)
		reg |= ((mfr & MFR_MASK) << MFR_SHIFT) | ((mrr & MRR_MASK) << MRR_SHIFT);
	writel(reg, cmu_top + PLL_CON5_PLL_MMC);

	/* setting SSCG_EN */
	reg = readl(cmu_top + PLL_CON4_PLL_MMC);
	if (ssc_on)
		reg |= 1 << SSCG_EN;
	else
		reg &= ~(1 << SSCG_EN);
	writel(reg, cmu_top + PLL_CON4_PLL_MMC);

	/* enable PLL_MMC */
	ret = pll_mmc_enable(1);
	if (ret)
		pr_err("%s: pll_mmc_enable failed\n", __func__);

	return ret;
}
EXPORT_SYMBOL(cal_pll_mmc_set_ssc);

void s5e5515_cal_data_init(void)
{
	int i;

	pr_info("%s: cal data init\n", __func__);

	/* cpu inform sfr initialize */
	pmucal_sys_powermode[SYS_SICD] = CPU_INFORM_SICD;
	pmucal_sys_powermode[SYS_SLEEP] = CPU_INFORM_SLEEP;

	cpu_inform_c2 = CPU_INFORM_C2;
	cpu_inform_cpd = CPU_INFORM_CPD;

	cmu_top = ioremap(s5e5515_CMU_TOP_BASE, SZ_4K);
	if (!cmu_top)
		pr_err("%s: cmu_top ioremap failed\n", __func__);

//	cmu_bus0 = ioremap(s5e5515_CMU_BUS0_BASE, SZ_16K);
//	if (!cmu_bus0)
//		pr_err("%s: cmu_bus0 ioremap failed\n", __func__);

	cmu_alive = ioremap(s5e5515_CMU_ALIVE_BASE, SZ_8K);
	if (!cmu_alive)
		pr_err("%s: cmu_alive ioremap failed\n", __func__);

//	ccmu_cpucl2 = ioremap(s5e5515_CCMU_CPUCL2_BASE, SZ_4K);
//	if (!ccmu_cpucl2)
//		pr_err("%s: ccmu_cpucl2 ioremap failed\n", __func__);

	frac_rpll_size = ARRAY_SIZE(s5e5515_frac_rpll_list);
	for (i = 0; i < frac_rpll_size; i++)
		frac_rpll_list[i] = s5e5515_frac_rpll_list[i];

//	cmu_adc_rco_400_contorl();
}

void (*cal_data_init)(void) = s5e5515_cal_data_init;

#if IS_ENABLED(CONFIG_SEC_FACTORY)
int asv_ids_information(enum ids_info id)
{
	int res;

	switch (id) {
	case tg:
		res = asv_get_table_ver();
		break;
	case lg:
		res = asv_get_grp(CPUCL0);
		break;
	case mifg:
		res = asv_get_grp(MIF);
		break;
	case intg:
		res = asv_get_grp(INT);
		break;
	case g3dg:
		res = asv_get_grp(G3D);
		break;
	case dispg:
		res = asv_get_grp(DISP);
		break;
	case audg:
		res = asv_get_grp(AUD);
		break;
	case lids:
		res = asv_get_ids_info(CPUCL0);
		break;
	case gids:
		res = asv_get_ids_info(G3D);
		break;
	case dvs:
		res = asv_get_ids_info(DVS);
		break;
	default:
		res = 0;
		break;
	};
	return res;
}
EXPORT_SYMBOL_GPL(asv_ids_information);
#endif

/*
static void __s5e5515_set_cmuewf(unsigned int index, unsigned int en, void *cmu_cmu)
{
	unsigned int reg;
	unsigned int reg_idx;

	if (index >= 32) {
		reg_idx = EARLY_WAKEUP_FORCED_ENABLE1;
		index = index - 32;
	} else {
		reg_idx = EARLY_WAKEUP_FORCED_ENABLE0;
	}

	if (en) {
		reg = __raw_readl(cmu_cmu + reg_idx);
		reg |= 1 << index;
		__raw_writel(reg, cmu_cmu + reg_idx);
	} else {
		reg = __raw_readl(cmu_cmu + reg_idx);
		reg &= ~(1 << index);
		__raw_writel(reg, cmu_cmu + reg_idx);
	}
}

int s5e5515_set_cmuewf(unsigned int index, unsigned int en, void *cmu_cmu, int *ewf_refcnt)
{
	unsigned int reg;
	int ret = 0;
	int tmp;

	if (en) {
		__s5e5515_set_cmuewf(index, en, cmu_cmu);

		reg = __raw_readl(cmu_bus0 + QCH_CON_TREX_D0_BUS0_QCH_OFFSET);
		reg |= 1 << IGNORE_FORCE_PM_EN;
		__raw_writel(reg, cmu_bus0 + QCH_CON_TREX_D0_BUS0_QCH_OFFSET);

		ewf_refcnt[index] += 1;
	} else {

		tmp = ewf_refcnt[index] - 1;

		if (tmp == 0) {
			reg = __raw_readl(cmu_bus0 + QCH_CON_TREX_D0_BUS0_QCH_OFFSET);
			reg &= ~(1 << IGNORE_FORCE_PM_EN);
			__raw_writel(reg, cmu_bus0 + QCH_CON_TREX_D0_BUS0_QCH_OFFSET);

			__s5e5515_set_cmuewf(index, en, cmu_cmu);

		} else if (tmp < 0) {
			pr_err("[EWF]%s ref count mismatch. ewf_index:%u\n",__func__,  index);

			ret = -EINVAL;
			goto exit;
		}

		ewf_refcnt[index] -= 1;
	}

exit:
	return ret;
}
int (*wa_set_cmuewf)(unsigned int index, unsigned int en, void *cmu_cmu, int *ewf_refcnt) = s5e5515_set_cmuewf;
*/  

void s5e5515_set_cmu_smpl_warn(void)
{
}
void (*cal_set_cmu_smpl_warn)(void) = s5e5515_set_cmu_smpl_warn;

bool is_ignore_cmu_dbg(u32 addr)
{
	int i;

	for (i = 0; i < NUM_SKIP_CMU_SFR; i++) {

		if (addr == skip_cmu_sfr[i])
			return true;
	}

	return false;
}

enum chub_sfr_enum {
	MEMORY_CTRL_CHUB_OUT,
	MEMORY_CTRL_CHUB_IN,
	CHUB_SFR_MAX,
};
static void __iomem *chub_sfr[CHUB_SFR_MAX];
static u32 watch_version;

void s5e5515_lpd_sram_cal_data_init(void)
{
	int ret;

	if (!lpd_sram_on_size || !lpd_sram_off_size) {
		return;
	}

	ret = pmucal_rae_phy2virt(lpd_sram_on, lpd_sram_on_size);
	if (ret) {
		pr_err("%s %s: error on PA2VA conversion for lpd_sram_on seq. aborting init...\n",
				PMUCAL_PREFIX, __func__);
		return;
	}

	ret = pmucal_rae_phy2virt(lpd_sram_off, lpd_sram_off_size);
	if (ret) {
		pr_err("%s %s: error on PA2VA conversion for lpd_sram_off seq. aborting init...\n",
				PMUCAL_PREFIX, __func__);
		return;
	}

	watch_version = cal_watch_version();

	chub_sfr[MEMORY_CTRL_CHUB_OUT] = ioremap(0x12863fa0, SZ_4);
	chub_sfr[MEMORY_CTRL_CHUB_IN] = ioremap(0x12863fa4, SZ_4);

	if (watch_version == 5 || watch_version == 6) {
		__raw_writel(0xff000070, chub_sfr[MEMORY_CTRL_CHUB_OUT]);
	} else {
		__raw_writel(0xff0000f0, chub_sfr[MEMORY_CTRL_CHUB_OUT]);
	}

}
void (*lpd_sram_cal_data_init)(void) = s5e5515_lpd_sram_cal_data_init;

void s5e5515_lpd_sram_power_control(unsigned int enable)
{
	u32 reg, timeout;

	if (enable) {
		// PGEN clear
		reg = __raw_readl(chub_sfr[MEMORY_CTRL_CHUB_OUT]);
		reg = (reg & ~(0x10 << 0)) | ((0x0 << 0) & (0x10 << 0));
		__raw_writel(reg, chub_sfr[MEMORY_CTRL_CHUB_OUT]);

		// PGEN__FEEDBACK
		timeout = 0;
		while(1) {
			if ((__raw_readl(chub_sfr[MEMORY_CTRL_CHUB_IN]) & (0x10 << 0)) == (0x0 << 0))
				break;
			timeout++;
			udelay(1);
			if (timeout > 200000) {
				pr_err("%s :timed out during wait. MEMORY_CTRL_CHUB_IN  (value:0x%x)\n",
						__func__, __raw_readl(chub_sfr[MEMORY_CTRL_CHUB_IN]));
			}
		}
		// RET:16 clear
		reg = __raw_readl(chub_sfr[MEMORY_CTRL_CHUB_OUT]);
		reg = (reg & ~(0x10 << 16)) | ((0x0 << 16) & (0x10 << 16));
		__raw_writel(reg, chub_sfr[MEMORY_CTRL_CHUB_OUT]);

		// CEN:8 clear
		reg = __raw_readl(chub_sfr[MEMORY_CTRL_CHUB_OUT]);
		reg = (reg & ~(0x10 << 8)) | ((0x0 << 8) & (0x10 << 8));
		__raw_writel(reg, chub_sfr[MEMORY_CTRL_CHUB_OUT]);

	} else {
		if (watch_version == 5 || watch_version == 6) {
			// PGEN set
			reg = __raw_readl(chub_sfr[MEMORY_CTRL_CHUB_OUT]);
			reg = (reg & ~(0x70 << 0)) | ((0x70 << 0) & (0x70 << 0));
			__raw_writel(reg, chub_sfr[MEMORY_CTRL_CHUB_OUT]);

			// PGEN__FEEDBACK
			timeout = 0;
			while(1) {
				if ((__raw_readl(chub_sfr[MEMORY_CTRL_CHUB_IN]) & (0x70 << 0)) == (0x70 << 0))
					break;
				timeout++;
				udelay(1);
				if (timeout > 200000) {
					pr_err("%s :timed out during wait. MEMORY_CTRL_CHUB_IN  (value:0x%x)\n",
							__func__, __raw_readl(chub_sfr[MEMORY_CTRL_CHUB_IN]));
				}
			}
		} else {
			// PGEN set
			reg = __raw_readl(chub_sfr[MEMORY_CTRL_CHUB_OUT]);
			reg = (reg & ~(0xf0 << 0)) | ((0xf0 << 0) & (0xf0 << 0));
			__raw_writel(reg, chub_sfr[MEMORY_CTRL_CHUB_OUT]);

			// PGEN__FEEDBACK
			timeout = 0;
			while(1) {
				if ((__raw_readl(chub_sfr[MEMORY_CTRL_CHUB_IN]) & (0xf0 << 0)) == (0xf0 << 0))
					break;
				timeout++;
				udelay(1);
				if (timeout > 200000) {
					pr_err("%s :timed out during wait. MEMORY_CTRL_CHUB_IN  (value:0x%x)\n",
							__func__, __raw_readl(chub_sfr[MEMORY_CTRL_CHUB_IN]));
				}
			}
		}
	}
}
void (*lpd_sram_power_control)(unsigned int enable) = s5e5515_lpd_sram_power_control;
EXPORT_SYMBOL(lpd_sram_power_control);

/*
char *s5e5515_get_pd_name_by_cmu(unsigned int addr)
{
	int i, map_size;

	map_size = (sizeof(cmu_pmu_map) / sizeof(struct cmu_pmu));
	for (i = 0; i < map_size; i++) {
		if (cmu_pmu_map[i].cmu == addr)
			break;
	}

	if (i < map_size)
		return cmu_pmu_map[i].pmu;
	else
		return NULL;
}
char *(*cal_get_pd_name_by_cmu)(unsigned int addr) = s5e5515_get_pd_name_by_cmu;
*/

