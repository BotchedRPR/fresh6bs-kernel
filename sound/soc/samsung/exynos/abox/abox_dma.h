/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ALSA SoC - Samsung Abox DMA driver
 *
 * Copyright (c) 2019 Samsung Electronics Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SND_SOC_ABOX_DMA_H
#define __SND_SOC_ABOX_DMA_H

#include <linux/completion.h>
#include <sound/soc.h>
#include "abox_ion.h"
#include "abox_soc.h"
#include "abox.h"
#include "abox_compress.h"

#define DMA_REG_CTRL0		0x00
#define DMA_REG_CTRL		DMA_REG_CTRL0
#define DMA_REG_CTRL1		0x04
#define DMA_REG_BUF_STR		0x08
#define DMA_REG_BUF_END		0x0c
#define DMA_REG_BUF_OFFSET	0x10
#define DMA_REG_STR_POINT	0x14
#define DMA_REG_VOL_FACTOR	0x18
#define DMA_REG_VOL_CHANGE	0x1c
#define DMA_REG_SBANK_LIMIT	0x20
#define DMA_REG_BIT_CTRL	0x24
#define DMA_REG_DITHER_SEED	0x28
#define DMA_REG_STATUS		0x30
#define DMA_REG_MAX		DMA_REG_STATUS

#define DMA_VOL_STATUS_SHIFT	0x0
#define DMA_VOL_STATUS_MASK	0x7

/* mask for field which are controlled by kernel in shared sfr */
#define REG_CTRL_KERNEL_MASK (ABOX_DMA_SYNC_MODE_MASK | \
		ABOX_DMA_BURST_LEN_MASK | ABOX_DMA_FUNC_MASK | \
		ABOX_DMA_AUTO_FADE_IN_MASK | ABOX_DMA_DUMMY_START_MASK)

#define BUFFER_ION_BYTES_MAX		(SZ_512K)

#define ABOX_DMA_SINGLE_S(xname, xreg, xshift, xmax, xsign_bit, xinvert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, \
	.get = abox_dma_mixer_control_get, .put = abox_dma_mixer_control_put, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
	{.reg = xreg, .rreg = xreg, .shift = xshift, .rshift = xshift, \
	 .max = xmax, .platform_max = xmax, .sign_bit = xsign_bit,} }

enum abox_platform_type {
	PLATFORM_NORMAL,
	PLATFORM_CALL,
	PLATFORM_COMPRESS,
	PLATFORM_REALTIME,
	PLATFORM_VI_SENSING,
	PLATFORM_SYNC,
};

enum abox_buffer_type {
	BUFFER_TYPE_DMA,
	BUFFER_TYPE_ION,
	BUFFER_TYPE_RAM,
};

enum abox_rate {
	RATE_SUHQA,
	RATE_UHQA,
	RATE_NORMAL,
	RATE_COUNT,
};

enum abox_dma_irq {
	DMA_IRQ_BUF_DONE,
	DMA_IRQ_BUF_FULL = DMA_IRQ_BUF_DONE,
	DMA_IRQ_BUF_EMPTY = DMA_IRQ_BUF_DONE,
	DMA_IRQ_FADE_DONE,
	DMA_IRQ_ERR,
	DMA_IRQ_COUNT,
};

enum abox_dma_dai {
	DMA_DAI_PCM,
	DMA_DAI_BE,
	DMA_DAI_COUNT,
};

enum abox_dma_param {
	DMA_RATE,
	DMA_WIDTH,
	DMA_CHANNEL,
	DMA_PERIOD,
	DMA_PERIODS,
	DMA_PACKED,
	DMA_PARAM_COUNT,
};

struct abox_dma_dump {
	struct proc_dir_entry *file;
	wait_queue_head_t waitqueue;
	void *area;
	phys_addr_t addr;
	size_t bytes;
	size_t pointer;
	bool updated;
	atomic_t open_state;
};

struct abox_dma_data {
	struct device *dev;
	void __iomem *sfr_base;
	void __iomem *mailbox_base;
	phys_addr_t sfr_phys;
	unsigned int id;
	unsigned int pointer;
	int pm_qos_cl0[RATE_COUNT];
	int pm_qos_cl1[RATE_COUNT];
	int pm_qos_cl2[RATE_COUNT];
	unsigned int sbank_size;
	struct device *dev_abox;
	struct abox_data *abox_data;
	struct snd_pcm_substream *substream;
	enum abox_platform_type type;
	struct snd_dma_buffer dmab;
	struct snd_dma_buffer ramb;
	struct abox_ion_buf *ion_buf;
	struct snd_hwdep *hwdep;
	enum abox_buffer_type buf_type;
	bool enabled;
	bool ack_enabled;
	bool backend;
	bool closing;
	bool auto_fade_in;
	struct completion closed;
	struct completion func_changed;
	unsigned int c_reg_ctrl; /* cache for dma_ctrl */
	struct abox_compr_data compr_data;
	struct regmap *mailbox;
	struct snd_soc_component *cmpnt;
	struct snd_soc_dai_driver *dai_drv;
	unsigned int num_dai;
	struct snd_pcm_hw_params hw_params;
	const struct abox_dma_of_data *of_data;
	struct miscdevice misc_dev;
	struct abox_dma_dump *dump;
	unsigned int dma_reg_max;
};

struct abox_dma_of_data {
	enum abox_irq (*get_irq)(struct abox_dma_data *data,
			enum abox_dma_irq irq);
	enum abox_dai (*get_dai_id)(enum abox_dma_dai dai, int id);
	char *(*get_dai_name)(struct device *dev, enum abox_dma_dai dai,
			int id);
	char *(*get_str_name)(struct device *dev, int id, int stream);
	enum abox_widget (*get_src_widget)(struct abox_dma_data *data);
	const struct snd_soc_dai_driver *dai_drv;
	unsigned int num_dai;
	const struct snd_soc_component_driver *cmpnt_drv;
};

extern const struct snd_soc_component_driver abox_dma;
extern const struct soc_enum abox_dma_func_enum;

/**
 * Get sampling rate type
 * @param[in]	rate		sampling rate in Hz
 * @return	rate type in enum abox_rate
 */
static inline enum abox_rate abox_get_rate_type(unsigned int rate)
{
	if (rate < 176400)
		return RATE_NORMAL;
	else if (rate >= 176400 && rate <= 192000)
		return RATE_UHQA;
	else
		return RATE_SUHQA;
}

/**
 * Check whether a dma is in sync mode. Only valid with rdma and wdma.
 * @param[in]	data		data of dma
 * @return	true or false
 */
extern bool abox_dma_is_sync_mode(struct abox_dma_data *data);

/**
 * Get IO virtual address of a dma
 * @param[in]	data		data of dma
 * @return	IO virtual address
 */
extern unsigned int abox_dma_iova(struct abox_dma_data *data);

/**
 * Get irq number of a dma irq
 * @param[in]	data		data of dma
 * @param[in]	irq		dma irq
 * @return	irq number
 */
extern enum abox_irq abox_dma_get_irq(struct abox_dma_data *data,
		enum abox_dma_irq irq);

/**
 * Enable DMA irq and set target to AP
 * @param[in]	data		data of dma
 * @param[in]	irq		dma irq
 */
extern void abox_dma_acquire_irq(struct abox_dma_data *data,
		enum abox_dma_irq dma_irq);

/**
 * Disable DMA irq and return target to ABOX core0
 * @param[in]	data		data of dma
 * @param[in]	irq		dma irq
 */
extern void abox_dma_release_irq(struct abox_dma_data *data,
		enum abox_dma_irq dma_irq);

/**
 * Register DMA irq handler
 * @param[in]	data		data of dma
 * @param[in]	irq		dma irq
 * @param[in]	handler		irq handler
 * @param[in]	dev_id		private data
 * @return	0 or error code
 */
extern int abox_dma_register_irq(struct abox_dma_data *data,
		enum abox_dma_irq irq, irq_handler_t handler, void *dev_id);

/**
 * Unregister DMA irq handler
 * @param[in]	data		data of dma
 * @param[in]	irq		dma irq
 */
extern void abox_dma_unregister_irq(struct abox_dma_data *data,
		enum abox_dma_irq irq);

/**
 * Shared callback for mixer type kcontrol get
 * @param[in]	kcontrol	kcontrol
 * @param[out]	ucontrol	ucontrol
 * @return	0 or error code
 */
extern int abox_dma_mixer_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol);

/**
 * Shared callback for enum type kcontrol put
 * @param[in]	kcontrol	kcontrol
 * @param[in]	ucontrol	ucontrol
 * @return	0 or error code
 */
extern int abox_dma_mixer_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol);

/**
 * Wait for DMA stable
 * @param[in]	dev		pointer to abox_dma device
 * @param[in]	data		data of dma
 * @param[in]	enable		enable or disable
 * @return	0 or error code
 */
extern void abox_dma_barrier(struct device *dev, struct abox_dma_data *data,
		int enable);

/**
 * read dma pointer value
 * @param[in]	data		data of dma
 * @return	dma pointer
 */
extern unsigned int abox_dma_read_pointer(struct abox_dma_data *data);

/**
 * Set destination bit width of dma.
 * @param[in]	dev		pointer to abox_dma device
 * @param[in]	width		bit width
 * @return	0 or error code
 */
extern int abox_dma_set_dst_bit_width(struct device *dev, int width);

/**
 * Get destination bit width of dma.
 * @param[in]	dev		pointer to abox_dma device
 * @return	bit width
 */
extern int abox_dma_get_dst_bit_width(struct device *dev);

/**
 * Get count of channels of dma.
 * @param[in]	dev		pointer to abox_dma device
 * @return	count of channels
 */
extern int abox_dma_get_channels(struct device *dev);

/**
 * fixup hardware parameter of the dma
 * @param[in]	dev		pointer to abox_dma device
 * @param[in]	params		hardware parameter
 * @return	0 or error code
 */
extern int abox_dma_hw_params_fixup(struct device *dev,
		struct snd_pcm_hw_params *params);

/**
 * set hardware parameter of the dma
 * @param[in]	dev		pointer to abox_dma device
 * @param[in]	rate		sampling rate
 * @param[in]	width		bit width
 * @param[in]	channel		channel count
 * @param[in]	period_size	number of frames in period
 * @param[in]	periods		number of period
 * @param[in]	packed		true for 24bit in three bytes format
 * @return	0 or error code
 */
extern void abox_dma_hw_params_set(struct device *dev, unsigned int rate,
		unsigned int width, unsigned int channels,
		unsigned int period_size, unsigned int periods, bool packed);

/**
 * Add DMA hw params controls to the given component
 * @param[in]	cmpnt		target component
 * @return	0 or error code
 */
extern int abox_dma_add_hw_params_controls(struct snd_soc_component *cmpnt);

/**
 * Add DMA function controls to the given component
 * @param[in]	cmpnt		target component
 * @return	0 or error code
 */
extern int abox_dma_add_func_controls(struct snd_soc_component *cmpnt);

/**
 * Get dai of the dma
 * @param[in]	dev		pointer to abox_dma device
 * @param[in]	type		type of the dai
 * @return	dai
 */
extern struct snd_soc_dai *abox_dma_get_dai(struct device *dev,
		enum abox_dma_dai type);

/**
 * Test dma can be closed
 * @param[in]	rtd	asoc runtime
 * @param[in]	stream	stream direction
 * @return	true or false
 */
extern int abox_dma_can_close(struct snd_soc_pcm_runtime *rtd, int stream);

/**
 * Test dma can be freed
 * @param[in]	rtd	asoc runtime
 * @param[in]	stream	stream direction
 * @return	true or false
 */
extern int abox_dma_can_free(struct snd_soc_pcm_runtime *rtd, int stream);

/**
 * Test dma can be stopped
 * @param[in]	rtd	asoc runtime
 * @param[in]	stream	stream direction
 * @return	true or false
 */
extern int abox_dma_can_stop(struct snd_soc_pcm_runtime *rtd, int stream);

/**
 * Test dma can be started
 * @param[in]	rtd	asoc runtime
 * @param[in]	stream	stream direction
 * @return	true or false
 */
extern int abox_dma_can_start(struct snd_soc_pcm_runtime *rtd, int stream);

/**
 * Test dma can be prepared
 * @param[in]	rtd	asoc runtime
 * @param[in]	stream	stream direction
 * @return	true or false
 */
extern int abox_dma_can_prepare(struct snd_soc_pcm_runtime *rtd, int stream);

/**
 * Test dma can be configured
 * @param[in]	rtd	asoc runtime
 * @param[in]	stream	stream direction
 * @return	true or false
 */
extern int abox_dma_can_params(struct snd_soc_pcm_runtime *rtd, int stream);

/**
 * Test dma can be opened
 * @param[in]	rtd	asoc runtime
 * @param[in]	stream	stream direction
 * @return	true or false
 */
extern int abox_dma_can_open(struct snd_soc_pcm_runtime *rtd, int stream);

/**
 * Check whether dma is opened
 * @param[in]	dev		pointer to abox_dma device
 * @return	true or false
 */
extern bool abox_dma_is_opened(struct device *dev);

#endif /* __SND_SOC_ABOX_DMA_H */
