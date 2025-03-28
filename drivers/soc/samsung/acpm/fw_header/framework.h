/*
 * framework.h - Header for the ACPM framework
 *
 * author : Yongjin Lee (yongjin0.lee@samsung.com)
 *	    Jeonghoon Jang (jnghn.jang@samsung.com)
 */

#ifndef __FRAMEWORK_H_
#define __FRAMEWORK_H_

#if IS_ENABLED(CONFIG_EXYNOS_ACPM)
#include "common.h"
#endif

/**
 * struct acpm_framework - General information for ACPM framework.
 *
 * @plugins:		Pointer to soc-specific plugins array.
 * @pid_framework:	Plugin ID for ACPM framework.
 * @pid_max:		# of plugins.
 * @ktime_index:	ktime information from the kernel.
 * @log_buf_rear:	Rear pointer of the log buffer.
 * @log_buf_front:	Front pointer of the log buffer.
 * @log_data:		Base address of the log buffer.
 * @log_entry_size:	Entry size of the log buffer.
 * @log_entry_len:	Length of the log buffer.
 * @ipc_base:		Base address of the IPC buffer.
 * @intr_to_skip:	Disabled interrupts.
 * @preemption_irq:	preemptive interrupts.
 * @intr_flag_offset:	Field offset for Mailbox interrupt pending register.
 */
struct acpm_framework {
#if !IS_ENABLED(CONFIG_EXYNOS_ACPM)
	struct plugin *plugins;
#else
	u32 plugins;
#endif
	u32 num_plugins;
#if !IS_ENABLED(CONFIG_EXYNOS_ACPM)
	struct ipc_channel *ipc_channels;
#else
	u32 ipc_channels;
#endif
#if defined(CONFIG_SOC_S5E9935) || defined(CONFIG_SOC_S5E8535) || defined(CONFIG_SOC_S5E8835)
	u8 num_ipc_channels;
#else
	u32 num_ipc_channels;
#endif
	u32 pid_framework;
	u32 pid_max;
#if defined(CONFIG_SOC_S5E9935) || defined(CONFIG_SOC_S5E8535) || defined(CONFIG_SOC_S5E8835)
	u8 ipc_ap_max;
#else
	u32 ipc_ap_max;
#endif
	u32 ktime_index;
	u32 log_buf_rear;
	u32 log_buf_front;
	u32 log_data;
	u32 log_entry_size;
	u32 log_entry_len;
	u32 ipc_base;
#if !(defined(CONFIG_SOC_S5E9935) || defined(CONFIG_SOC_S5E8535) || defined(CONFIG_SOC_S5E8835))
	u32 ipc_buf_tx_size;
#endif
#if defined(CONFIG_SOC_S5E9935) || defined(CONFIG_SOC_S5E8535) || defined(CONFIG_SOC_S5E8835)
	u8 intr_flag_offset;
#else
	u32 intr_flag_offset;
#endif
	struct build_info info;
	u32 nvic_max;
	u32 preempt_plugin;
#if !(defined(CONFIG_SOC_S5E9935) || defined(CONFIG_SOC_S5E8535) || defined(CONFIG_SOC_S5E8835))
	u32 preempt_log_buf_rear;
	u32 preempt_log_buf_front;
	u32 preempt_log_data;
	u32 preempt_log_entry_len;
#endif
	unsigned long long timestamps[32];
	u32 board_info;
#if !(defined(CONFIG_SOC_S5E9935) || defined(CONFIG_SOC_S5E8535) || defined(CONFIG_SOC_S5E8835))
#if !IS_ENABLED(CONFIG_EXYNOS_ACPM)
	struct acpm_interrupt *p_intr_vector;
	u32 *p_irq_max;
	u32 *p_intr_en;
	u32 *p_intr_pri_filter;
#else
	u32 p_intr_vector;
	u32 p_irq_max;
	u32 p_intr_en;
	u32 p_intr_pri_filter;
#endif
#endif
#if (defined(CONFIG_SOC_S5E9935) || defined(CONFIG_SOC_S5E8535) || defined(CONFIG_SOC_S5E8835))
	bool task_profile_running;
	u32 task_info;
#endif
};

/**
 * struct channel_info - IPC information of a channel
 *
 * @rx_rear:		Rear ptr for RX cmd queue. (for dequeue)
 * @rx_front:		Front ptr for RX cmd queue. (just for empty check)
 * @rx_base:		Predefined base addr for RX cmd queue.
 * @tx_rear:		Rear ptr for TX queue.
 * @tx_front:		Front ptr for TX queue.
 * @tx_base:		Predefined base addr for TX queue.
 * @q_len:		Length of both TX/RX queue.
 * @q_elem_size:	Element size of both TX/RX queue.
 * @credit:		For preventing starvation of certain plugin.
 */
struct channel_info {
#if defined(CONFIG_SOC_S5E9935) || defined(CONFIG_SOC_S5E8535) || defined(CONFIG_SOC_S5E8835)
	u32 mbox_addr;
	u32 rx_rear;
	u32 rx_front;
	u32 rx_base;
	u32 tx_rear;
	u32 tx_front;
	u32 tx_base;
	u8 q_len;
	u8 q_elem_size;
	u8 credit;
	bool skip_qfull_chk;
#else
	u32 rx_rear;
	u32 rx_front;
	u32 rx_base;
	u32 rx_indr_buf;
	u32 rx_indr_buf_base;
	u32 rx_indr_buf_size;
	u32 tx_rear;
	u32 tx_front;
	u32 tx_base;
	u32 q_len;
	u32 q_elem_size;
	u32 credit;
	u32 mbox_addr;
#endif
};

/**
 * struct ipc_channel - descriptor for ipc channel.
 *
 * @id:			The ipc channel's id.
 * @field:		The ipc channel's field in mailbox status register.
 * @owner:		This interrupt's Belonged plugin.
 * @type:		The ipc channel's memory type; QUEUE or Register.
 * @ch:			IPC information for this plugin's channel.
 * @ap_poll:		The flag indicating whether AP polls on this channel or not. (interrupt)
 */
struct ipc_channel {
#if (defined(CONFIG_SOC_S5E9935) || defined(CONFIG_SOC_S5E8535) || defined(CONFIG_SOC_S5E8835))
	u8 id;
	u32 field;
	u8 owner;
	u8 type;
	struct channel_info ch;
	bool ap_poll;
#else
	u32 id;
	u32 field;
	s32 owner;
	u32 type;
	struct channel_info ch;
	u32 ap_poll;
#endif
};

/**
 * struct acpm_interrupt - descriptor for individual interrupt source.
 *
 * @field:		The interrupt's field in NVIC pending register.
 * @handler:		Handler for this interrupt.
 * @owner:		This interrupt's Belonged plugin.
 * @is_disabled:	Flag to skip this interrupt or not.
 */
struct acpm_interrupt {
	u32 field;
	s32 (*handler)(u32 intr);
	s32 owner;
	bool is_disabled;
	bool log_skip;
};

/**
 * struct apm_peri - The abstraction for APM peripherals.
 *
 * @mpu_init:		Initializes Core's MPU. (Regions, ...)
 * @enable_systick:	Enables System Tick.
 * @disable_systick:	Disables System Tick.
 * @get_systick_cvr:	Get the current value register of System Tick.
 * @get_systick_icvr:	Get (MAX_VAL - CUR_VAL) of System Tick.
 * @get_systick_icvra:	Get (MAX_VAL - CUR_VAL) & MAX_VAL of System Tick.
 * @get_systick_csr:	Get the current status register of System Tick.
 * @enable_wdt:		Enables Watchdog Timer.
 * @disable_wdt:	Disables Watchdog Timer.
 * @enable_intr:	Enables specific interrupts on NVIC.
 * @get_enabled_intr:	Get enabled interrupts on NVIC.
 * @clr_pend_intr:	Clears pended interrupts on NVIC.
 * @get_pend_intr:	Get Pended interrupts on NVIC.
 * @get_mbox_pend_intr:	Get Pended interrupts on Mailbox.
 * @clr_mbox_pend_intr:	Clears pended interrupts on Mailbox.
 * @gen_mbox_intr_ap:	Generates Mailbox interrupt to AP.
 */
struct apm_peri {
	void (*power_mode_init) (void);
	void (*peri_timer_init) (void);
	void (*set_peri_timer_event) (u32 msec);
	void (*del_peri_timer_event) (void);
	u32 (*get_peri_timer_cvr) (void);
	u32 (*get_peri_timer_icvr) (void);
	u32 (*get_peri_timer_icvra) (void);
	void (*mpu_init) (void);
	void (*enable_systick) (void);
	void (*disable_systick) (void);
	u32 (*get_systick_cvr) (void);
	u32 (*get_systick_icvr) (void);
	u32 (*get_systick_icvra) (void);
	u32 (*get_systick_csr) (void);
	void (*enable_wdt) (void);
	void (*disable_wdt) (void);
	void (*enable_intr) (u32 idx, u32 intr);
	void (*disable_intr) (u32 idx, u32 intr);
	u32 (*get_enabled_intr) (u32 idx);
	void (*clr_pend_intr) (u32 idx, u32 intr);
	void (*set_intr_priority) (u32 nvic_bit, u8 priority);
	u32 (*get_active_intr) (u32 idx);
	u32 (*get_pend_intr) (u32 idx);
	u32 (*get_mbox_addr) (u32 intr);
	u32 (*get_mbox_pend_intr) (u32 mbox_addr);
	void (*clr_mbox_pend_intr) (u32 mbox_addr, u32 intr);
	void (*gen_mbox_intr) (struct ipc_channel *ipc_ch);
	s32 (*udelay_systick) (u32 udelay_us);
	void (*set_wdtrst_req) (void);
};
extern struct apm_peri apm_peri;

/**
 * struct plat_ops - Getters for platform specific data to ACPM framework.
 *
 * @plugin_base_init:		initializes base address of plugins.
 * @get_acpm_platform_data:	inits the remaining data.
 * @get_target_plugin:		returns next plugin to be executed.
 */
struct plat_ops {
	void (*plugin_base_init) (struct acpm_framework *acpm);
	void (*get_acpm_platform_data) (struct acpm_framework *acpm);
	s32 (*get_target_plugin) (u32 intr, u32 int_status, u32 *ch_num);
	void (*ipc_channel_init) (struct acpm_framework *acpm);
	void *(*get_plugins_extern_fn) (void);
	void (*wait_for_intr) (struct acpm_framework *acpm);
	u32 (*system_reset) (void);
	void (*preemption_enable) (void);
	void (*preemption_disable) (void);
	void (*preemption_disable_irq_save)(u32 *flag);
	void (*preemption_enable_irq_restore)(u32 *flag);
	u32 (*get_board_type)(void);
	u32 (*get_board_rev)(void);
	u32 (*get_chip_rev)(void);
};
extern struct plat_ops plat_ops;

s32 acpm_generic_ipc_handler(u32 intr);
void acpm_insert_dbg_log(u32 plugin_id, struct dbg_log_info *log);
extern void acpm_print(u32 id, const char *s, u32 int_data, u32 level);
extern struct acpm_interrupt *get_intr_vector(u32 *irq_max, u8 priority);
extern struct acpm_interrupt *get_preempt_intr_vector(u32 *irq_max);
extern u32 get_nvic_max(void);
extern struct acpm_framework acpm;

enum shared_buffer_type {
	TYPE_QUEUE = 1,
	TYPE_BUFFER,
};

struct acpm_list_head {
	u32 next, prev;
};

struct acpm_work_queue {
	struct acpm_list_head queue_head;
	u32 works;
	u32 front;
	u32 rear;
	u32 max_entries;
	u32 num_entries;
};

struct logger_info {
	u32 buf_rear;
	u32 buf_front;
	u32 log_base;
	u32 log_entry_size;
	u32 log_entry_len;
};

struct task_profile {
	u32 start;
	u32 latency;
};

struct acpm_task_struct {
	u32 sp;
	u32 sp_lim;
	u8 status;
	u8 id;
#if (defined(CONFIG_SOC_S5E9935) || defined(CONFIG_SOC_S5E8535) || defined(CONFIG_SOC_S5E8835))
	u8 dl_warn_status;
#endif
	struct acpm_work_queue wq;
	struct logger_info logbuff;
	struct task_profile profile;
};

struct task_info {
	u32 tasks;
	u8 num_tasks;
	u8 task_id_idle;
#if (defined(CONFIG_SOC_S5E9935) || defined(CONFIG_SOC_S5E8535) || defined(CONFIG_SOC_S5E8835))
	u32 dl_warn_us;
#endif
	void (*platform_task_switch)(void);
	void (*platform_task_worker)(struct work_struct *work);
};

#define LOG_ID_SHIFT			(28)
#define LOG_TIME_INDEX			(20)
#define LOG_LEVEL			(19)
#define AVAILABLE_TIME			(26000)

#define ACPM_DEBUG_PRINT

#ifdef ACPM_DEBUG_PRINT
#define ACPM_PRINT_ERR(id, s, int_data)	acpm_print(id, s, int_data, 0xF)
#define ACPM_PRINT(id, s, int_data)	acpm_print(id, s, int_data, 0x1)
#else
#define ACPM_PRINT_ERR(id, s, int_data)	do {} while(0)
#define ACPM_PRINT(a, b, c) do {} while(0)
#endif

#define true				(1)
#define false				(0)

/* IPC Protocol bit field definitions */
#define IPC_PRTC_OWN			(31)
#define IPC_PRTC_RSP			(30)
#define IPC_PRTC_INDR			(29)
#define IPC_PRTC_ID			(26)
#define IPC_PLUGIN_ID			(4)
#define IPC_PRTC_IDX			(0x7 << IPC_PRTC_ID)
#define IPC_PLUGIN_IDX			(0xf << IPC_PLUGIN_ID)
#define IPC_PRTC_DP_ATTACH		(25)
#define IPC_PRTC_DP_DETACH		(24)
#define IPC_PRTC_DP_CMD			((1 << IPC_PRTC_DP_ATTACH) | (1 << IPC_PRTC_DP_DETACH))
#define IPC_PRTC_TEST			(23)
#define IPC_PRTC_STOP			(22)
#define IPC_PRTC_STOP_WDT		(27)

#define ERR_NOPLUGIN			(10)
#define ERR_REJECTED			(11)
#define ERR_NOTYET			(12)

#if !IS_ENABLED(CONFIG_EXYNOS_ACPM)
struct pmic_if_ops *acpm_pmic_if_ops;
#endif

#define pmic_if_init()	        		\
					acpm_pmic_if_ops->init()
#if defined(CONFIG_EXYNOS_MFD_I3C)
#define pmic_if_tx(channel, reg, val)		\
					acpm_pmic_if_ops->tx(channel, reg, val)
#define pmic_if_rx(channel, reg)		\
					acpm_pmic_if_ops->rx(channel, reg)
#elif defined(CONFIG_EXYNOS_MFD_SPMI)
#define pmic_if_tx(sid, reg, val)		\
					acpm_pmic_if_ops->tx(sid, reg, val)
#define pmic_if_rx(sid, reg)		\
					acpm_pmic_if_ops->rx(sid, reg)
#define pmic_if_bulk_tx(sid, reg, val, byte_count)		\
					acpm_pmic_if_ops->bulk_tx(sid, reg, val, byte_count)
#define pmic_if_bulk_rx(sid, reg, val, byte_count)		\
					acpm_pmic_if_ops->bulk_rx(sid, reg, val, byte_count)
#endif

#define MAGIC				0x0BAD0BAD

#endif
