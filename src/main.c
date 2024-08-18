/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Sample app for Audio class
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>

#include <nrfx_nvmc.h>

#include <nau8325.h>
#include <zephyr/drivers/i2s.h>

#include <sample_usbd.h>
#include "feedback.h"

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_uac2.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define HEADPHONES_OUT_TERMINAL_ID UAC2_ENTITY_ID(DT_NODELABEL(out_terminal))

#define SAMPLE_FREQUENCY    (SAMPLES_PER_SOF * 1000)
#define SAMPLE_BIT_WIDTH    16
#define NUMBER_OF_CHANNELS  2
#define BYTES_PER_SAMPLE    DIV_ROUND_UP(SAMPLE_BIT_WIDTH, 8)
#define BYTES_PER_SLOT      (BYTES_PER_SAMPLE * NUMBER_OF_CHANNELS)
#define MIN_BLOCK_SIZE      ((SAMPLES_PER_SOF - 1) * BYTES_PER_SLOT)
#define BLOCK_SIZE          (SAMPLES_PER_SOF * BYTES_PER_SLOT)
#define MAX_BLOCK_SIZE      ((SAMPLES_PER_SOF + 1) * BYTES_PER_SLOT)

/* Absolute minimum is 5 buffers (1 actively consumed by I2S, 2nd queued as next
 * buffer, 3rd acquired by USB stack to receive data to, and 2 to handle SOF/I2S
 * offset errors), but add 2 additional buffers to prevent out of memory errors
 * when USB host decides to perform rapid terminal enable/disable cycles.
 */
#define I2S_BUFFERS_COUNT   7
K_MEM_SLAB_DEFINE_STATIC(i2s_tx_slab, MAX_BLOCK_SIZE, I2S_BUFFERS_COUNT, 4);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;
static const struct device *const audio_output_dev = DEVICE_DT_GET(DT_INST(0, nuvoton_nau8325));
// static const struct device *const i2s_dev = DEVICE_DT_GET(DT_N_NODELABEL_i2s0);

// #define NUM_SAMPLES 48
// static int16_t data_l[NUM_SAMPLES] = {
// 	  6392,  12539,  18204,  23169,  27244,  30272,  32137,  32767,  32137,
// 	 30272,  27244,  23169,  18204,  12539,   6392,      0,  -6393, -12540,
// 	-18205, -23170, -27245, -30273, -32138, -32767, -32138, -30273, -27245,
// 	-23170, -18205, -12540,  -6393,     -1,
// };
// static int16_t data_r[NUM_SAMPLES] = {
// 	 12539,  23169,  30272,  32767,  30272,  23169,  12539,      0, -12540,
// 	-23170, -30273, -32767, -30273, -23170, -12540,     -1,  12539,  23169,
// 	 30272,  32767,  30272,  23169,  12539,      0, -12540, -23170, -30273,
// 	-32767, -30273, -23170, -12540,     -1,
// };
// #define BLOCK_SIZE (2 * sizeof(data_l))
// #define NUM_BLOCKS 5

#define NUM_VOLUME_LEVELS 5
static uint16_t volumes[NUM_VOLUME_LEVELS] = {1, 2, 4, 8, 16};
static uint8_t next_volume_level = 0;
static uint8_t current_volume_level = 0;

// 48 uint16 * 2 channels = 48 * 2 * 2 = 192 bytes
// static K_MEM_SLAB_DEFINE(mem_slab, BLOCK_SIZE, NUM_BLOCKS, 32);
// void *mem_blocks;

struct usb_i2s_ctx {
	const struct device *i2s_dev;
	bool terminal_enabled;
	bool i2s_started;
	/* Number of blocks written, used to determine when to start I2S.
	 * Overflows are not a problem becuse this variable is not necessary
	 * after I2S is started.
	 */
	uint8_t i2s_blocks_written;
	struct feedback_ctx *fb;
};

static void uac2_terminal_update_cb(const struct device *dev, uint8_t terminal,
				    bool enabled, bool microframes,
				    void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;

	/* This sample has only one terminal therefore the callback can simply
	 * ignore the terminal variable.
	 */
	__ASSERT_NO_MSG(terminal == HEADPHONES_OUT_TERMINAL_ID);
	/* This sample is for Full-Speed only devices. */
	__ASSERT_NO_MSG(microframes == false);

	ctx->terminal_enabled = enabled;
	if (ctx->i2s_started && !enabled) {
		i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
		ctx->i2s_started = false;
		ctx->i2s_blocks_written = 0;
		feedback_reset_ctx(ctx->fb);
	}
}

static void *uac2_get_recv_buf(const struct device *dev, uint8_t terminal,
			       uint16_t size, void *user_data)
{
	ARG_UNUSED(dev);
	struct usb_i2s_ctx *ctx = user_data;
	void *buf = NULL;
	int ret;

	if (terminal == HEADPHONES_OUT_TERMINAL_ID) {
		__ASSERT_NO_MSG(size <= MAX_BLOCK_SIZE);

		if (!ctx->terminal_enabled) {
			LOG_ERR("Buffer request on disabled terminal");
			return NULL;
		}

		ret = k_mem_slab_alloc(&i2s_tx_slab, &buf, K_NO_WAIT);
		if (ret != 0) {
			buf = NULL;
		}
	}

	return buf;
}

static void uac2_data_recv_cb(const struct device *dev, uint8_t terminal,
			      void *buf, uint16_t size, void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;
	int ret;

	if (next_volume_level != current_volume_level)
	{
		nau8325_write_reg(audio_output_dev, 0x63, volumes[next_volume_level]); // ANALOG_CONTROL_3, volume
		LOG_INF("Setting volume to: %d at %d", volumes[next_volume_level], next_volume_level);
		current_volume_level = next_volume_level;
	}

	if (!ctx->terminal_enabled) {
		k_mem_slab_free(&i2s_tx_slab, buf);
		return;
	}

	if (!size) {
		/* Zero fill to keep I2S going. If this is transient error, then
		 * this is probably best we can do. Otherwise, host will likely
		 * either disable terminal (or the cable will be disconnected)
		 * which will stop I2S.
		 */
		size = BLOCK_SIZE;
		memset(buf, 0, size);
		sys_cache_data_flush_range(buf, size);
	}

	LOG_DBG("Received %d data to input terminal %d", size, terminal);

	ret = i2s_write(ctx->i2s_dev, buf, size);
	if (ret < 0) {
		ctx->i2s_started = false;
		ctx->i2s_blocks_written = 0;
		feedback_reset_ctx(ctx->fb);

		/* Most likely underrun occurred, prepare I2S restart */
		i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE);

		ret = i2s_write(ctx->i2s_dev, buf, size);
		if (ret < 0) {
			/* Drop data block, will try again on next frame */
			k_mem_slab_free(&i2s_tx_slab, buf);
		}
	}

	if (ret == 0) {
		ctx->i2s_blocks_written++;
	}
}

static void uac2_buf_release_cb(const struct device *dev, uint8_t terminal,
				void *buf, void *user_data)
{
	/* This sample does not send audio data so this won't be called */
}

static volatile bool use_hardcoded_feedback;
static volatile uint32_t hardcoded_feedback = (48 << 14) + 1;

static uint32_t uac2_feedback_cb(const struct device *dev, uint8_t terminal,
				 void *user_data)
{
	/* Sample has only one UAC2 instance with one terminal so both can be
	 * ignored here.
	 */
	ARG_UNUSED(dev);
	ARG_UNUSED(terminal);
	struct usb_i2s_ctx *ctx = user_data;

	if (use_hardcoded_feedback) {
		return hardcoded_feedback;
	} else {
		return feedback_value(ctx->fb);
	}
}

static void uac2_sof(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	struct usb_i2s_ctx *ctx = user_data;

	if (ctx->i2s_started) {
		feedback_process(ctx->fb);
	}

	/* We want to maintain 3 SOFs delay, i.e. samples received during SOF n
	 * should be on I2S during SOF n+3. This provides enough wiggle room
	 * for software scheduling that effectively eliminates "buffers not
	 * provided in time" problem.
	 *
	 * ">= 2" translates into 3 SOFs delay because the timeline is:
	 * USB SOF n
	 *   OUT DATA0 n received from host
	 * USB SOF n+1
	 *   DATA0 n is available to UDC driver (See Universal Serial Bus
	 *   Specification Revision 2.0 5.12.5 Data Prebuffering) and copied
	 *   to I2S buffer before SOF n+2; i2s_blocks_written = 1
	 *   OUT DATA0 n+1 received from host
	 * USB SOF n+2
	 *   DATA0 n+1 is copied; i2s_block_written = 2
	 *   OUT DATA0 n+2 received from host
	 * USB SOF n+3
	 *   This function triggers I2S start
	 *   DATA0 n+2 is copied; i2s_block_written is no longer relevant
	 *   OUT DATA0 n+3 received from host
	 */
	if (!ctx->i2s_started && ctx->terminal_enabled &&
	    ctx->i2s_blocks_written >= 2) {
		i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
		ctx->i2s_started = true;
		feedback_start(ctx->fb, ctx->i2s_blocks_written);
	}
}

static struct uac2_ops usb_audio_ops = {
	.sof_cb = uac2_sof,
	.terminal_update_cb = uac2_terminal_update_cb,
	.get_recv_buf = uac2_get_recv_buf,
	.data_recv_cb = uac2_data_recv_cb,
	.buf_release_cb = uac2_buf_release_cb,
	.feedback_cb = uac2_feedback_cb,
};

static struct usb_i2s_ctx main_ctx;

#if 0

static void data_received(const struct device *dev,
			  struct net_buf *buffer,
			  size_t size)
{
	int ret;

	if (!buffer || !size) {
		/* This should never happen */
		return;
	}

	LOG_DBG("Received %d data, buffer %p", size, buffer);

	if (mem_blocks != NULL)
	{
		for (int i = 0; i < size; i++) {
			((uint8_t*)mem_blocks)[i] = buffer->data[i];
		}

		int res = 0;
		
		if (device_is_ready(i2s_dev))
		{
			res = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE);

			res = i2s_buf_write(i2s_dev, mem_blocks, BLOCK_SIZE);
			if (res < 0) {
				LOG_INF("Error: i2s_write failed with %d\n", res);
			}

			res = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
		}
	}
	
	net_buf_unref(buffer);

}

static void feature_update(const struct device *dev,
			   const struct usb_audio_fu_evt *evt)
{
	uint8_t val = 0;
	LOG_DBG("Control selector %d for channel %d updated",
		evt->cs, evt->channel);
	switch (evt->cs) {
	case USB_AUDIO_FU_MUTE_CONTROL:
		val = &evt->val;
		LOG_INF("Mute control: %d, len: %d", val, evt->val_len);
		break;
	case USB_AUDIO_FU_VOLUME_CONTROL:
		val = &evt->val;
		LOG_INF("Volume control: %d, len: %d", val, evt->val_len);
		break;
	default:
		break;
	}
}

static const struct usb_audio_ops hp_ops = {
	.data_received_cb = data_received,
	.feature_update_cb = feature_update,
};

static const struct usb_audio_ops mic_ops = {
	.feature_update_cb = feature_update,
};

#endif

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	next_volume_level = (current_volume_level + 1) % NUM_VOLUME_LEVELS;
}

int main(void)
{
	if (NRF_UICR->VREGHVOUT != UICR_VREGHVOUT_VREGHVOUT_3V3)
	{
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
		{
		}
		NRF_UICR->VREGHVOUT = UICR_VREGHVOUT_VREGHVOUT_3V3;

		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
		{
		}
	}

	if (!device_is_ready(led.port))
	{
		LOG_ERR("LED not ready");
		return 0;
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set_dt(&led, 0);
	k_msleep(100);
	gpio_pin_set_dt(&led, 1);
	k_msleep(50);
	gpio_pin_set_dt(&led, 0);
	k_msleep(25);
	gpio_pin_set_dt(&led, 1);
	k_msleep(12);
	gpio_pin_set_dt(&led, 0);
	k_msleep(5);
	gpio_pin_set_dt(&led, 1);
	k_msleep(1);
	gpio_pin_set_dt(&led, 0);

	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(uac2_headphones));
	struct usbd_context *sample_usbd;
	struct i2s_config config;
	int ret;
	
	main_ctx.i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s_tx));

	if (!device_is_ready(main_ctx.i2s_dev)) {
		printk("%s is not ready\n", main_ctx.i2s_dev->name);
		return 0;
	}

	config.word_size = SAMPLE_BIT_WIDTH;
	config.channels = NUMBER_OF_CHANNELS;
	config.format = I2S_FMT_DATA_FORMAT_I2S;
	config.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	config.frame_clk_freq = SAMPLE_FREQUENCY;
	config.mem_slab = &i2s_tx_slab;
	config.block_size = MAX_BLOCK_SIZE;
	config.timeout = 0;

	uint16_t value;

	int res = nau8325_write_reg(audio_output_dev, 0x0, 42); // Software Reset
	nau8325_read_reg(audio_output_dev, 0x2, &value);
	LOG_INF("Result: %d", value); // Should be 0x21F2, 8690 in decimal
	nau8325_write_reg(audio_output_dev, 0x0D, 0b0000000000000010); // 16 bit I2S
	nau8325_write_reg(audio_output_dev, 0x40, 0b1010100000000001); // DAC data does not gate power up
	nau8325_write_reg(audio_output_dev, 0x65, 0b0000000000000111); // enable 8x MCLK and extra multipliers
	nau8325_write_reg(audio_output_dev, 0x03, 0b0000000000100000); // MCLK x8
	nau8325_write_reg(audio_output_dev, 0x61, 0b0001010101010101); // Enable everything through clock detection
	nau8325_write_reg(audio_output_dev, 0x63, 0b0000000000000001); // ANALOG_CONTROL_3, volume
	nau8325_write_reg(audio_output_dev, 0x04, 0b0000000000001100); // Enable L and R DAC (required)
	nau8325_write_reg(audio_output_dev, 0x13, 0xf3f3); // DAC Volume
	nau8325_write_reg(audio_output_dev, 0x73, 0x0); // Unregulated DAC

	if (!gpio_is_ready_dt(&button))
	{
		LOG_INF("%s is not ready\n", button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		LOG_INF("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_INACTIVE);
	if (ret != 0) {
		LOG_INF("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	const struct device *i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s0));
	if (!device_is_ready(i2s_dev)) {
		LOG_INF("%s is not ready\n", i2s_dev->name);
		return 0;
	}

	config.word_size = SAMPLE_BIT_WIDTH;
	config.channels = NUMBER_OF_CHANNELS;
	config.format = I2S_FMT_DATA_FORMAT_I2S;
	config.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	config.frame_clk_freq = SAMPLE_FREQUENCY;
	config.mem_slab = &i2s_tx_slab;
	config.block_size = MAX_BLOCK_SIZE;
	config.timeout = 50;
	
	ret = i2s_configure(main_ctx.i2s_dev, I2S_DIR_TX, &config);
	if (ret < 0) {
		printk("Failed to configure TX stream: %d\n", ret);
		return 0;
	}

	main_ctx.fb = feedback_init();

	usbd_uac2_set_ops(dev, &usb_audio_ops, &main_ctx);

	LOG_INF("Initializing USBD");

	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		return -ENODEV;
	}

	ret = usbd_enable(sample_usbd);
	if (ret) {
		return ret;
	}

	LOG_INF("USBD initialized");

	next_volume_level = 0;

	return 0;
#if 0
	/* Configure the I2S device */
	struct i2s_config i2s_cfg;
	i2s_cfg.word_size = 16; // due to int16_t in data_frame declaration
	i2s_cfg.channels = 2; // L + R channel
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	i2s_cfg.frame_clk_freq = 48000;
	i2s_cfg.mem_slab = &mem_slab;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = 50;
	res = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
	if (res < 0) {
		LOG_INF("Failed to configure the I2S stream: (%d)\n", res);
		return 0;
	}

	LOG_INF("i2s configured");

	res = k_mem_slab_alloc(&mem_slab, &mem_blocks, K_NO_WAIT);
	if (res < 0) {
		LOG_INF("Failed to allocate the memory blocks: %d\n", res);
		return 0;
	}

	LOG_INF("mem slab allocd");

	memset((uint16_t*)mem_blocks, 0, NUM_SAMPLES * NUM_BLOCKS);

	LOG_INF("Initialized.");

	next_volume_level = 0;

	return 0;
#endif
}
