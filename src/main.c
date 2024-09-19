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

#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_audio.h>
#include <zephyr/usb/class/usb_hid.h>

#include <nrfx_nvmc.h>

#include <nau8325.h>
#include <zephyr/drivers/i2s.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// static const struct device *const mic_dev = DEVICE_DT_GET_ONE(usb_audio_mic);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;
static const struct device *const audio_output_dev = DEVICE_DT_GET(DT_INST(0, nuvoton_nau8325));
static const struct device *const i2s_dev = DEVICE_DT_GET(DT_N_NODELABEL_i2s0);

#define NUM_SAMPLES 48
static int16_t data_l[NUM_SAMPLES] = {
	  6392,  12539,  18204,  23169,  27244,  30272,  32137,  32767,  32137,
	 30272,  27244,  23169,  18204,  12539,   6392,      0,  -6393, -12540,
	-18205, -23170, -27245, -30273, -32138, -32767, -32138, -30273, -27245,
	-23170, -18205, -12540,  -6393,     -1,
};
static int16_t data_r[NUM_SAMPLES] = {
	 12539,  23169,  30272,  32767,  30272,  23169,  12539,      0, -12540,
	-23170, -30273, -32767, -30273, -23170, -12540,     -1,  12539,  23169,
	 30272,  32767,  30272,  23169,  12539,      0, -12540, -23170, -30273,
	-32767, -30273, -23170, -12540,     -1,
};
#define BLOCK_SIZE (2 * sizeof(data_l))
#define NUM_BLOCKS 5

#define NUM_VOLUME_LEVELS 5
static uint16_t volumes[NUM_VOLUME_LEVELS] = {1, 2, 4, 8, 16};
static uint8_t next_volume_level = 0;
static uint8_t current_volume_level = 0;

// 48 uint16 * 2 channels = 48 * 2 * 2 = 192 bytes
static K_MEM_SLAB_DEFINE(mem_slab, BLOCK_SIZE, NUM_BLOCKS, 32);
void *mem_blocks;

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

	if (next_volume_level != current_volume_level)
	{
		nau8325_write_reg(audio_output_dev, 0x63, volumes[next_volume_level]); // ANALOG_CONTROL_3, volume
		LOG_INF("Setting volume to: %d at %d", volumes[next_volume_level], next_volume_level);
		current_volume_level = next_volume_level;
	}
}

static void feature_update(const struct device *dev,
			   const struct usb_audio_fu_evt *evt)
{
	uint16_t val = 0;
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

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	next_volume_level = (current_volume_level + 1) % NUM_VOLUME_LEVELS;
}

int main(void)
{
#if 0 // nrf52840
	if (NRF_UICR->REGOUT0 != UICR_REGOUT0_VOUT_3V3)
	{
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
		{
		}
		NRF_UICR->REGOUT0 = UICR_REGOUT0_VOUT_3V3;

		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
		{
		}
	}
#else // nrf5340
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
#endif

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

	const struct device *const hp_dev = DEVICE_DT_GET_ONE(usb_audio_hp);
	int ret;

	LOG_INF("Entered %s", __func__);

	if (!device_is_ready(hp_dev)) {
		LOG_ERR("Device USB Headphones is not ready");
		return 0;
	}

	LOG_INF("Found USB Headphones Device");

	usb_audio_register(hp_dev, &hp_ops);

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
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

	if (!device_is_ready(audio_output_dev)) {
		LOG_ERR("Could not initialize audio device!");
	}

	uint16_t value;

	int res = nau8325_write_reg(audio_output_dev, 0x0, 42); // Software Reset
	nau8325_read_reg(audio_output_dev, 0x2, &value);
	LOG_INF("Result: %d", value); // Should be 0x21F2
	nau8325_write_reg(audio_output_dev, 0x0D, 0b0000000000000010); // 16 bit I2S
	nau8325_write_reg(audio_output_dev, 0x40, 0b1010100000000001); // DAC data does not gate power up
	nau8325_write_reg(audio_output_dev, 0x65, 0b0000000000000111); // enable 8x MCLK and extra multipliers
	nau8325_write_reg(audio_output_dev, 0x03, 0b0000000000100000); // MCLK x8
	nau8325_write_reg(audio_output_dev, 0x61, 0b0001010101010101); // Enable everything through clock detection
	nau8325_write_reg(audio_output_dev, 0x63, 0b0000000000000001); // ANALOG_CONTROL_3, volume
	nau8325_write_reg(audio_output_dev, 0x04, 0b0000000000001100); // Enable L and R DAC (required)
	nau8325_write_reg(audio_output_dev, 0x13, 0xf3f3); // DAC Volume
	nau8325_write_reg(audio_output_dev, 0x73, 0x0); // Unregulated DAC

	LOG_INF("USB enabled");

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
}
