/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __UART_SEL_H__
#define __UART_SEL_H__

#define LOG_TAG	"usel: "

enum connect_type {
	USB = 0,
	UART = 1,
};

enum uart_direction_t {
	AP = 0,
	CP = 1,
};

struct uart_sel_data {
	struct device *dev;
	char *name;

	bool uart_connect;
	bool uart_switch_sel;

#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
	struct notifier_block uart_notifier;
	muic_attached_dev_t attached_dev;
#endif
	unsigned int int_uart_noti;
	/*unsigned int mbx_uart_noti;*/
	unsigned int mbx_ap_united_status;
	unsigned int sbi_uart_noti_mask;
	unsigned int sbi_uart_noti_pos;
	unsigned int use_usb_phy;
};

int get_uart_dir(void);


#define usel_err(fmt, ...) \
	pr_err(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)


#endif
