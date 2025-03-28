/*
 *
 * Zinitix ztm730 touch driver
 *
 * Copyright (C) 2018 Samsung Electronics Co.Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/miscdevice.h>
#if IS_ENABLED(CONFIG_PM_RUNTIME)
#include <linux/pm_runtime.h>
#endif
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/async.h>

#include "../../../sec_input/sec_input.h"
#include "../../../sec_input/sec_input_rawdata.h"

#if 0
//#if IS_ENABLED(CONFIG_BATTERY_SAMSUNG)
extern unsigned int lpcharge;
#endif

#define	ZTM730_VENDOR_NAME	"ZINITIX"
#define	TC_SECTOR_SZ		8
#define	I2C_RESUME_DELAY	2000
#define	WAKELOCK_TIME		200

#define WET_MODE_VALUE	0x0101
#define NOR_MODE_VALUE	0x0100

static struct ztm730_info *g_ztm730_info;

#define	ZTM730_IC_CHIP_CODE	0xE730
#define	ZTM630_IC_CHIP_CODE	0xE630
#define	ZTM620_IC_CHIP_CODE	0xE628
#define	ZTW522_IC_CHIP_CODE	0xE532
#define	ZT7538_IC_CHIP_CODE	0xE538


#define	ZTM620_IC_FW_SIZE	(48 * 1024)
#define	ZTM630_IC_FW_SIZE	(48 * 1024)
#define	ZTM730_IC_FW_SIZE	(64 * 1024)

#define	SEC_FACTORY_TEST

#define	SEC_TSP_NAME			"sec_touchscreen"
#define BEZEL_NAME			"detent_bezel"
#define	ZTM730_TS_DEVICE		"ztm730_ts"

#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
#define	ZINITIX_MISC_DEBUG
#define	ZINITIX_FILE_DEBUG
#endif

#ifdef ZINITIX_MISC_DEBUG
static struct ztm730_info *misc_info;
#endif
#ifdef ZINITIX_MISC_DEBUG
#define	ZINITIX_DEBUG			0
#endif

#define	TOUCH_POINT_MODE		0
#define	CHECK_HWID			1
#define	TSP_INIT_TEST_RATIO		100
#define	MAX_SUPPORTED_FINGER_NUM	2	/* max 10 */
#ifdef CONFIG_SEC_FACTORY
#define	SUPPORTED_PALM_TOUCH	0
#else
#define	SUPPORTED_PALM_TOUCH	1
#endif

#if IS_ENABLED(CONFIG_INPUT_TOUCHSCREEN_BEZEL)
#define	SUPPORTED_TOUCH_WHEEL	1
#else
#define	SUPPORTED_TOUCH_WHEEL	0
#endif

#define	SUPPORTED_FWUPGRADE_ERASE_PROGRAM		1

#define	REAL_Z_MAX			3000

#ifdef SEC_FACTORY_TEST
typedef void (*run_func_t)(void *);
enum data_type {
	DATA_UNSIGNED_CHAR,
	DATA_SIGNED_CHAR,
	DATA_UNSIGNED_SHORT,
	DATA_SIGNED_SHORT,
	DATA_UNSIGNED_INT,
	DATA_SIGNED_INT,
};

#define	COVER_OPEN			0
#define	COVER_CLOSED			3
#endif

/* resolution offset */
#define	ABS_PT_OFFSET			(-1)
#define	TOUCH_FORCE_UPGRADE		0
#define	USE_CHECKSUM			1
#define	CHIP_OFF_DELAY			50	/*ms*/
#define	DELAY_FOR_SIGNAL_DELAY	30	/*us*/
#define	DELAY_FOR_TRANSCATION		50
#define	DELAY_FOR_POST_TRANSCATION	15
#define	RAWDATA_DELAY_FOR_HOST	100
#define	CHECKSUM_VAL_SIZE		2

#if USE_CHECKSUM
#define	ZTM730_CHECK_SUM		0x55aa
#endif

/*Test Mode (Monitoring Raw Data) */
#define	TSP_INIT_TEST_RATIO		100
#define	SEC_MUTUAL_AMP_V_SEL		0x0232

#define	CONNECT_TEST_DELAY		100
#define	I2C_TX_DELAY			1
#define	I2C_START_DELAY		500
#define	GPIO_GET_DELAY		1
#define	LPM_OFF_DELAY			1	/*ms*/
#define	POWER_ON_DELAY		50	/*ms*/
#define	RESET_DELAY			5	/*ms*/
#define	CHIP_ON_DELAY			50	/*ms*/
#define	FIRMWARE_ON_DELAY		100	/*ms*/
#define	SEC_DND_N_COUNT		11
#define	SEC_DND_U_COUNT		16
#define	SEC_DND_FREQUENCY		139

#define	SEC_HFDND_N_COUNT		11
#define	SEC_HFDND_U_COUNT		16
#define	SEC_HFDND_FREQUENCY		104

#define	SEC_M_U_COUNT		   3
#define	SEC_M_N_COUNT		   11
#define	SEC_M_FREQUENCY		 84
#define	SEC_M_RST0_TIME		 170

#define	SEC_SX_AMP_V_SEL		0x0434
#define	SEC_SX_SUB_V_SEL		0x0055
#define	SEC_SY_AMP_V_SEL		0x0232
#define	SEC_SY_SUB_V_SEL		0x0022
#define	SEC_SHORT_N_COUNT		2
#define	SEC_SHORT_U_COUNT		1

//DND
#define	SEC_DND_CP_CTRL_L		0x1fb3
#define	SEC_DND_V_FORCE		0
#define	SEC_DND_AMP_V_SEL		0x0141

#define	MAX_LIMIT_CNT			10000
#define	MAX_RAW_DATA_SZ		36*22
#define	MAX_TRAW_DATA_SZ	\
	(MAX_RAW_DATA_SZ + 4*MAX_SUPPORTED_FINGER_NUM + 2)

#define	TOUCH_DELTA_MODE		3
#define	TOUCH_PROCESS_MODE		4
#define	TOUCH_NORMAL_MODE		5
#define	TOUCH_CND_MODE		6
#define	TOUCH_REFERENCE_MODE		8
#define	TOUCH_REF_MODE			10
#define	TOUCH_DND_MODE		11
#define	TOUCH_HFDND_MODE		12
#define	TOUCH_TRXSHORT_MODE		13
#define TOUCH_JITTER_MODE		15
#define	TOUCH_SFR_MODE			17
#define TOUCH_HYBRID_MODE		20
#define TOUCH_CHARGE_PUMP_MODE	25
#define	TOUCH_SSR_MODE			39
#define	TOUCH_SEC_MODE			48
#define	TOUCH_JITTER_ACCELERATION_MODE			50
#define	TOUCH_NO_OPERATION_MODE  55

#define	PALM_REPORT_WIDTH		200
#define	PALM_REJECT_WIDTH		255

#define	TOUCH_V_FLIP			0x01
#define	TOUCH_H_FLIP			0x02
#define	TOUCH_XY_SWAP			0x04

#define	TSP_NORMAL_EVENT_MSG		1
#define	I2C_RETRY_TIMES			2
#define	I2C_START_RETRY_TIMES		5
#define	I2C_BUFFER_SIZE			64
#define	INT_CLEAR_RETRY			5
#define	CONNECT_TEST_RETRY		2
/*  Other Things */
#define	INIT_RETRY_CNT			3
#define	I2C_SUCCESS			0
#define	I2C_FAIL				1

#if SUPPORTED_TOUCH_WHEEL
#define	BIT_WHEEL_EVENT		7
#define	WHEEL_BIT_PRESS		4
#define	WHEEL_BIT_RELEASE	5
#define	WHEEL_BIT_CW		6
#define	WHEEL_BIT_CCW		7
#define DEF_OPTIONAL_MODE_WHEEL_ON_BIT	14
#define	TOUCH_WHEEL_MODE	16
#define	ZTM730_WHEEL_VIB_SETTING		0x0037
#define BIT_WHEEL_VIB_MODE_0	8
#define BIT_WHEEL_VIB_MODE_1	9
#define MASK_WHEEL_VIB_MODE	3

enum {
	VIB_NONE,
	VIB_LIGHT,
	VIB_MEDIUM,
	VIB_STRONG,
};
#endif

#define	INTERNAL_FLAG_01_DEFAULT	0x4021
#define	INTERNAL_FLAG_02_DEFAULT	0x0012

#define	TOUCH_IOCTL_BASE			0xbc
#define	TOUCH_IOCTL_GET_DEBUGMSG_STATE	_IOW(TOUCH_IOCTL_BASE, 0, int)
#define	TOUCH_IOCTL_SET_DEBUGMSG_STATE	_IOW(TOUCH_IOCTL_BASE, 1, int)
#define	TOUCH_IOCTL_GET_CHIP_REVISION		_IOW(TOUCH_IOCTL_BASE, 2, int)
#define	TOUCH_IOCTL_GET_FW_VERSION		_IOW(TOUCH_IOCTL_BASE, 3, int)
#define	TOUCH_IOCTL_GET_REG_DATA_VERSION	_IOW(TOUCH_IOCTL_BASE, 4, int)
#define	TOUCH_IOCTL_VARIFY_UPGRADE_SIZE	_IOW(TOUCH_IOCTL_BASE, 5, int)
#define	TOUCH_IOCTL_VARIFY_UPGRADE_DATA	_IOW(TOUCH_IOCTL_BASE, 6, int)
#define	TOUCH_IOCTL_START_UPGRADE		_IOW(TOUCH_IOCTL_BASE, 7, int)
#define	TOUCH_IOCTL_GET_X_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 8, int)
#define	TOUCH_IOCTL_GET_Y_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 9, int)
#define	TOUCH_IOCTL_GET_TOTAL_NODE_NUM	_IOW(TOUCH_IOCTL_BASE, 10, int)
#define	TOUCH_IOCTL_SET_RAW_DATA_MODE	_IOW(TOUCH_IOCTL_BASE, 11, int)
#define	TOUCH_IOCTL_GET_RAW_DATA		_IOW(TOUCH_IOCTL_BASE, 12, int)
#define	TOUCH_IOCTL_GET_X_RESOLUTION		_IOW(TOUCH_IOCTL_BASE, 13, int)
#define	TOUCH_IOCTL_GET_Y_RESOLUTION		_IOW(TOUCH_IOCTL_BASE, 14, int)
#define	TOUCH_IOCTL_HW_CALIBRAION		_IOW(TOUCH_IOCTL_BASE, 15, int)
#define	TOUCH_IOCTL_GET_REG			_IOW(TOUCH_IOCTL_BASE, 16, int)
#define	TOUCH_IOCTL_SET_REG			_IOW(TOUCH_IOCTL_BASE, 17, int)
#define	TOUCH_IOCTL_SEND_SAVE_STATUS		_IOW(TOUCH_IOCTL_BASE, 18, int)
#define	TOUCH_IOCTL_DONOT_TOUCH_EVENT	_IOW(TOUCH_IOCTL_BASE, 19, int)
#define	TOUCH_IOCTL_ZI_FILE_DEBUG_DISABLE	_IOW(TOUCH_IOCTL_BASE, 20, int)

/* Register Map*/
#define	ZTM730_SWRESET_CMD				0x0000
#define	ZTM730_WAKEUP_CMD				0x0001
#define	ZTM730_DEEP_SLEEP_CMD			0x0004
#define	ZTM730_SLEEP_CMD				0x0005
#define	ZTM730_CLEAR_INT_STATUS_CMD			0x0003
#define	ZTM730_CALIBRATE_CMD				0x0006
#define	ZTM730_SAVE_STATUS_CMD			0x0007
#define	ZTM730_SAVE_CALIBRATION_CMD			0x0008
//#define	ZTM730_I2C_START_CMD				0x000A
//#define	ZTM730_I2C_END_CMD	  			0x000B
#define	ZTM730_DRIVE_INT_STATUS_CMD 0x000C
#define	ZTM730_RECALL_FACTORY_CMD			0x000f
#define	ZTM730_THRESHOLD				0x0020
#define	ZTM730_DEBUG_REG				0x0115 /* 0~7 */
#define	ZTM730_TOUCH_MODE				0x0010
#define	ZTM730_CHIP_REVISION				0x0011
#define	ZTM730_FIRMWARE_VERSION			0x0012
#define	ZTM730_MINOR_FW_VERSION			0x0121
#define	ZTM730_VENDOR_ID				0x001C
#define	ZTM730_HW_ID					0x0014
#define	ZTM730_DATA_VERSION_REG			0x0013
#define	ZTM730_SUPPORTED_FINGER_NUM			0x0015
#define	ZTM730_EEPROM_INFO				0x0018
#define	ZTM730_INITIAL_TOUCH_MODE			0x0019
#define ZTM730_JITTER_SAMPLING_CNT		0x001F
#define	ZTM730_TOTAL_NUMBER_OF_X			0x0060
#define	ZTM730_TOTAL_NUMBER_OF_Y			0x0061
#define	ZTM730_DELAY_RAW_FOR_HOST			0x007f
#define	ZTM730_BUTTON_SUPPORTED_NUM			0x00B0
#define	ZTM730_BUTTON_SENSITIVITY			0x00B2
#define	ZTM730_DUMMY_BUTTON_SENSITIVITY		0X00C8
#define	ZTM730_X_RESOLUTION				0x00C0
#define	ZTM730_Y_RESOLUTION				0x00C1
#define	ZTM730_COORD_ORIENTATION		0x00C2
#define	ZTM730_POINT_STATUS_REG			0x0080
#define	ZTM730_ICON_STATUS_REG			0x00AA
#define	ZTM730_I2C_MODE_REG				0x00E2

#define	ZTM730_WIRELESS_CHARGER_MODE		0x0199
#define ZTM730_POWER_STATE_FLAG			0x007E
#define	ZTM730_GESTURE_STATE				0x01FB
#define	ZTM730_MUTUAL_AMP_V_SEL			0x02F9
#define	ZTM730_DND_SHIFT_VALUE			0x012B
#define	ZTM730_AFE_FREQUENCY				0x0100
#define	ZTM730_DND_N_COUNT				0x0122
#define	ZTM730_DND_U_COUNT				0x0135
#define	ZTM730_CONNECTED_REG				0x0062
#define	ZTM730_CONNECTED_REG_MAX			0x0078
#define	ZTM730_CONNECTED_REG_MIN			0x0079
#define	ZTM730_DND_V_FORCE				0x02F1
#define	ZTM730_DND_AMP_V_SEL				0x02F9
#define	ZTM730_DND_CP_CTRL_L				0x02bd
#define	ZTM730_RAWDATA_REG				0x0200
#define	ZTM730_EEPROM_INFO_REG			0x0018
#define	ZTM730_INT_ENABLE_FLAG			0x00f0
#define	ZTM730_PERIODICAL_INTERRUPT_INTERVAL	0x00f1
#define	ZTM730_BTN_WIDTH				0x0316
#define	ZTM730_CHECKSUM_RESULT			0x012c
#define	ZTM730_INIT_FLASH				0x01d0
#define	ZTM730_WRITE_FLASH				0x01d1
#define	ZTM730_READ_FLASH				0x01d2
#define	ZINITIX_INTERNAL_FLAG_01			0x011d
#define	ZINITIX_INTERNAL_FLAG_02			0x011e
#define	ZTM730_OPTIONAL_SETTING			0x0116
#define	ZT75XX_SX_AMP_V_SEL				0x02DF
#define	ZT75XX_SX_SUB_V_SEL				0x02E0
#define	ZT75XX_SY_AMP_V_SEL				0x02EC
#define	ZT75XX_SY_SUB_V_SEL				0x02ED

#define	ZTM730_M_U_COUNT				0x02F2
#define	ZTM730_M_N_COUNT				0x02F3
#define	ZTM730_M_RST0_TIME	  			0x02F5
#define	ZTM730_REAL_WIDTH				0x03A6
#define	ZTM730_FW_CHECKSUM_REG  			0x03DF
#define	ZTM730_FINGER_CNT_REG  			0x030F
#define	ZTM730_REPORT_RAWDATA  			0x016A
#define	ZTM730_DEBUG_00				0x0115

/* Interrupt & status register flag bit
-------------------------------------------------
*/
#define	BIT_PT_CNT_CHANGE	0
#define	BIT_DOWN		1
#define	BIT_MOVE		2
#define	BIT_UP			3
#define	BIT_PALM		4
#define	BIT_PALM_REJECT	5
#define	BIT_GESTURE_WAKE	6
#define	RESERVED_1		7
#define	BIT_WEIGHT_CHANGE	8
#define	BIT_PT_NO_CHANGE	9
#define	BIT_REJECT		10
#define	BIT_PT_EXIST		11
#define	RESERVED_2		12
#define	BIT_MUST_ZERO		13
#define	BIT_DEBUG		14
#define	BIT_ICON_EVENT		15


/* DEBUG_00 */
#define	BIT_DEVICE_STATUS_LPM	0
#define	BIT_POWER_SAVING_MODE	14

#define DEF_DEVICE_STATUS_NPM				0// read only
#define DEF_DEVICE_STATUS_WALLET_COVER_MODE	1// read only
#define DEF_DEVICE_STATUS_NOISE_MODE			2// read only
#define DEF_DEVICE_STATUS_WATER_MODE		3// read only
#define DEF_DEVICE_STATUS_LPM				4// read only
#define DEF_DEVICE_STATUS_GLOVE_MODE			5// read only
#define DEF_DEVICE_STATUS_HIGH_SEN_MODE		6// read only
#define DEF_DEVICE_STATUS_HIGH_SEN_NORMAL_MODE	7// read only

#define DEF_DEVICE_STATUS_MAX_SKIP			8// read only
#define DEF_DEVICE_STATUS_NOISE_SKIP			9// read only
#define DEF_DEVICE_STATUS_PALM_DETECT		10// read only
#define DEF_DEVICE_STATUS_SVIEW_MODE		11// read only
#define DEF_DEVICE_STATUS_ESD_COVER			12
#define DEF_DEVICE_STATUS_BODY_MODE			13

/* button */
#define	BIT_O_ICON0_DOWN	0
#define	BIT_O_ICON1_DOWN	1
#define	BIT_O_ICON2_DOWN	2
#define	BIT_O_ICON3_DOWN	3
#define	BIT_O_ICON4_DOWN	4
#define	BIT_O_ICON5_DOWN	5
#define	BIT_O_ICON6_DOWN	6
#define	BIT_O_ICON7_DOWN	7

#define	BIT_O_ICON0_UP		8
#define	BIT_O_ICON1_UP		9
#define	BIT_O_ICON2_UP		10
#define	BIT_O_ICON3_UP		11
#define	BIT_O_ICON4_UP		12
#define	BIT_O_ICON5_UP		13
#define	BIT_O_ICON6_UP		14
#define	BIT_O_ICON7_UP		15

#define	SUB_BIT_EXIST		0
#define	SUB_BIT_DOWN		1
#define	SUB_BIT_MOVE		2
#define	SUB_BIT_UP		3
#define	SUB_BIT_UPDATE		4
#define	SUB_BIT_WAIT		5

#define	zinitix_bit_set(val, n)	((val) &= ~(1<<(n)), (val) |= (1<<(n)))
#define	zinitix_bit_clr(val, n)	((val) &= ~(1<<(n)))
#define	zinitix_bit_test(val, n)	((val) & (1<<(n)))
#define	zinitix_swap_v(a, b, t)	((t) = (a), (a) = (b), (b) = (t))
#define	zinitix_swap_16(s)	(((((s) & 0xff) << 8) | (((s) >> 8) & 0xff)))

/* REG_USB_STATUS : optional setting from AP */
#define	DEF_OPTIONAL_MODE_USB_DETECT_BIT		0
#define	DEF_OPTIONAL_MODE_SVIEW_DETECT_BIT		1
#define	DEF_OPTIONAL_MODE_SENSITIVE_BIT		2
#define	DEF_OPTIONAL_MODE_EDGE_SELECT		3
#define	DEF_OPTIONAL_MODE_DUO_TOUCH		4
#define	DEF_OPTIONAL_MODE_BODY_DETECTION	12

#ifdef SEC_FACTORY_TEST
/* Touch Screen */
#define	TSP_CMD_STR_LEN		32
#define	TSP_CMD_RESULT_STR_LEN	4096
#define	TSP_CMD_PARAM_NUM	8
#define	TSP_CMD_Y_NUM		18
#define	TSP_CMD_X_NUM		30
#define	TSP_CMD_NODE_NUM	(TSP_CMD_Y_NUM * TSP_CMD_X_NUM)
#define	TSP_CONNECTED_INVALID	0
#define	TSP_CONNECTED_VALID	1
#define	REG_EDGE_XF_OFFSET	0xEC
#define	REG_EDGE_XL_OFFSET	0xED
#define	REG_EDGE_YF_OFFSET	0xEE
#define	REG_EDGE_YL_OFFSET	0xEF

#define	REG_SET_I2C_NORMAL	610
#define	REG_SET_I2C_LPM		1600

/* ZINITIX_TS_DEBUG : Print event contents */
#define ZINITIX_REG_ADDR_LENGTH	2
//#define ZINITIX_TS_DEBUG_PRINT_ALLEVENT		0x01
//#define ZINITIX_TS_DEBUG_PRINT_ONEEVENT		0x02
#define ZINITIX_TS_DEBUG_PRINT_I2C_READ_CMD	0x04
#define ZINITIX_TS_DEBUG_PRINT_I2C_WRITE_CMD	0x08
//#define ZINITIX_TS_DEBUG_SEND_UEVENT		0x80

enum {
	WAITING = 0,
	RUNNING,
	OK,
	FAIL,
	NOT_APPLICABLE,
};
#endif

enum power_control {
	POWER_OFF,
	POWER_ON,
	POWER_ON_SEQUENCE,
};

enum key_event {
	ICON_BUTTON_UNCHANGE,
	ICON_BUTTON_DOWN,
	ICON_BUTTON_UP,
};

enum work_state {
	NOTHING = 0,
	NORMAL,
	ESD_TIMER,
	EALRY_SUSPEND,
	SUSPEND,
	RESUME,
	LATE_RESUME,
	UPGRADE,
	REMOVE,
	SET_MODE,
	HW_CALIBRAION,
	RAW_DATA,
	PROBE,
	SLEEP_MODE_IN,
	SLEEP_MODE_OUT,
};

enum {
	BUILT_IN = 0,
	UMS,
	FFU,
};

struct raw_ioctl {
	u32 sz;
	u32 buf;
};

struct reg_ioctl {
	u32 addr;
	u32 val;
};

struct coord {
	u16	x;
	u16	y;
	u8	width;
	u8	sub_status;
	u16	real_width;
};

struct point_info {
	u16 status;
#if TOUCH_POINT_MODE
	u16 	event_flag;
#else
	u8 finger_cnt;
	u8 time_stamp;
#endif
	struct coord coord[MAX_SUPPORTED_FINGER_NUM];
	u16 fw_status;
};

struct capa_info {
	u16	vendor_id;
	u16	ic_revision;
	u16	fw_version;
	u16	fw_minor_version;
	u16	reg_data_version;
	u16	threshold;
	u16	key_threshold;
	u16	dummy_threshold;
	u32	ic_fw_size;
	u32	MaxX;
	u32	MaxY;
	u32	MinX;
	u32	MinY;
	u8	gesture_support;
	u16	multi_fingers;
	u16	button_num;
	u16	ic_int_mask;
	u16	x_node_num;
	u16	y_node_num;
	u16	total_node_num;
	u16	hw_id;
	u16	afe_frequency;
	u16	shift_value;
	u16	v_force;
	u16	amp_v_sel;
	u16	N_cnt;
	u16	u_cnt;
	u16	cp_ctrl_l;
	u16	mutual_amp_v_sel;
	u16	i2s_checksum;
	u16	sx_amp_v_sel;
	u16	sx_sub_v_sel;
	u16	sy_amp_v_sel;
	u16	sy_sub_v_sel;
	u16 current_touch_mode;
};

struct zxt_ts_platform_data {
	int		gpio_int;
	int		vdd_en;
	int		gpio_scl;
	int		gpio_sda;
	int		gpio_ldo_en;
	int		gpio_reset;
	int		gpio_lpm;
	int		orientation;
	u32		x_resolution;
	u32		y_resolution;
	u32		page_size;
	u32		tsp_supply_type;
	u32		core_num;
	const char	*fw_name;
	const char	*model_name;
	bool		regulator_boot_on;
	const char 	*avdd_ldo;
	const char 	*dvdd_ldo;
	bool support_bezel_detent;
	int bringup;
	bool use_deep_sleep;
	bool enable_settings_aot;
	bool enable_sysinput_enabled;
};

struct finger_event {
	u32	x;
	u32	y;
	u32	move_cnt;
};

#if SUPPORTED_TOUCH_WHEEL
enum direction_patten {
	BZ_CC	= -1,
	BZ_CW	= 1,
};

struct ztm_bezel_ddata {
	struct input_dev		*input_dev;
	struct device			*rotary_dev;
};
#endif

struct ztm730_info {
	struct i2c_client			*client;
	struct input_dev			*input_dev;
	struct zxt_ts_platform_data		*pdata;
	struct sec_cmd_data sec;
#if SUPPORTED_TOUCH_WHEEL
	struct ztm_bezel_ddata		*bezel_info;
#endif

	struct capa_info			cap_info;
	struct point_info			touch_info;
	struct point_info			reported_touch_info;
	struct semaphore			work_lock;
	struct mutex			set_reg_lock;
	struct mutex modechange;
	struct semaphore			raw_data_lock;
	struct pinctrl			*pinctrl;
	struct pinctrl_state		*gpio_wake;
	struct pinctrl_state		*gpio_off;

	struct completion resume_done;
	struct wakeup_source *wakelock;

	struct finger_event		event[MAX_SUPPORTED_FINGER_NUM];
	struct regulator			*vddo;
	struct regulator			*avdd;
	struct device			*tsp_dev;
	wait_queue_head_t			wait_q;
	enum work_state			work_state;
	u32				finger_event_cnt;
	bool 				wait_until_wake;
	u16				touch_mode;
	s16				cur_data[MAX_TRAW_DATA_SZ];
	char				phys[32];
	u8				finger_cnt;
	u8				update;
	unsigned char			*fw_data;
	u16				icon_event_reg;
	u16				prev_icon_event;
	u16				chip_code;
	int				irq;
	s16				ref_scale_factor;
	s16				ref_btn_option;
	bool				device_enabled;
	u16				cover_state;
	u16				optional_mode;
	u16				m_prev_optional_mode;
	int				cal_mode;
	u8				lpm_mode;
	u32				multi_touch_cnt;
	u32				wet_mode_cnt;
	u32				comm_err_cnt;
	u32				hard_recovery_cnt;
	u32				noise_mode_cnt;
	u32				all_touch_cnt;
	u32				aod_touch_cnt;
	bool				wet_mode;
	bool				noise_mode;
	bool			body_mode;
	u32				max_z_value;
	int				ta_connected;
	u16				wireless_charger_mode;
	u8				palm_flag;
	struct tsp_raw_data		*raw_data;
	u16				checksum;
	u32 		recovery_cnt;
	u8 debug_flag;
//	volatile int power_status;
	bool probe_done;
	u8 bezel_enable;
	u8 aot_enable;
	u8 glove_mode;
	u8 power_state;
	int enabled;
	bool fix_active_mode;

	struct delayed_work work_print_info;
	struct delayed_work work_read_info;
	u32 print_info_cnt_open;
	u32 print_info_cnt_release;
	bool info_work_done;
	volatile bool shutdown_is_on_going;

	u16 img_version_of_bin[4];

	char raw_mode;
	int raw_len;
	u8 *raw_u8;//read from IC
	s16 *raw;//convert x/y

	int pressed;
	u16 store_reg_data;

#ifdef ZINITIX_FILE_DEBUG
	u16 g_zini_file_debug_disable;
	u32 g_zini_raw_data_size;
#endif
};

struct tsp_raw_data {
	s16	dnd_data[TSP_CMD_NODE_NUM];
	s16	vgap_data[TSP_CMD_NODE_NUM];
	s16	hgap_data[TSP_CMD_NODE_NUM];
	s16	trxshort_data[TSP_CMD_NODE_NUM];
	s16 ssr_data[TSP_CMD_NODE_NUM];
	s16 acc_test_data[TSP_CMD_NODE_NUM];
	s16 charge_pump_data[TSP_CMD_NODE_NUM];
	s16 jitter_data[TSP_CMD_NODE_NUM];
};

bool ztm730_power_control(struct ztm730_info *info, u8 ctl);
int ztm730_gpio_reset(struct ztm730_info *info);
int ztm730_ts_set_lowpowermode(struct ztm730_info *info, u8 mode);
int ztm730_ts_start(struct ztm730_info *info);
int ztm730_ts_stop(struct ztm730_info *info);
#if SUPPORTED_TOUCH_WHEEL
int ztm730_bezel_open(struct input_dev *dev);
void ztm730_bezel_close(struct input_dev *dev);

#endif

bool ztm730_init_touch(struct ztm730_info *info);
bool ztm730_mini_init_touch(struct ztm730_info *info);
void ztm730_clear_report_data(struct ztm730_info *info);
#if IS_ENABLED(CONFIG_OF)
int ztm730_pinctrl_configure(struct ztm730_info *info);
#endif

bool ztm730_hw_calibration(struct ztm730_info *info);
s32 ztm730_write_reg(struct i2c_client *client, u16 reg, u16 value);
s32 ztm730_write_cmd(struct i2c_client *client, u16 reg);
s32 ztm730_read_data(struct i2c_client *client, u16 reg, u8 *values, u16 length);
int ztm730_soft_reset(struct ztm730_info *info);

bool ztm730_upgrade_firmware(struct ztm730_info *info, const u8 *firmware_data, u32 size);
s32 ztm730_read_raw_data(struct i2c_client *client, u16 reg, u8 *values, u16 length);
int ztm730_set_optional_mode(struct ztm730_info *info, bool force);
int ztm730_mode_change_recovery(struct ztm730_info *info);
int  ztm730_write_wakeup_cmd(struct ztm730_info *info);
int ztm730_hard_reset_recovery(struct ztm730_info *info);
int ztm730_hard_reset(struct ztm730_info *info);
void ztm730_input_close(struct input_dev *dev);
int ztm730_input_open(struct input_dev *dev);
void set_gesture_mode(struct ztm730_info *info, bool enable);

bool ztm730_set_touchmode(struct ztm730_info *info, u16 value);
int ztm730_upgrade_sequence(struct ztm730_info *info, const u8 *firmware_data);
int read_fw_verify_result(struct ztm730_info *info);

int init_sec_factory(struct ztm730_info *info);
void remove_sec_factory(struct ztm730_info *info);
int ztm730_ic_version_check(struct ztm730_info *info);
int ztm730_fix_active_mode(struct ztm730_info *info, bool active);
void ztm730_enable_irq(struct ztm730_info *info, bool enable);
void ztm730_run_rawdata_all(struct ztm730_info *info);

#if USE_CHECKSUM
bool crc_check(struct ztm730_info *info);
#endif