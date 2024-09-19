
#include "audio_i2s.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <nrfx_i2s.h>
#include <nrfx_clock.h>

#include "audio_sync_timer.h"

#define I2S_NL DT_NODELABEL(i2s0)

#define HFCLKAUDIO_12_288_MHZ 0x9BAE

enum audio_i2s_state {
	AUDIO_I2S_STATE_UNINIT,
	AUDIO_I2S_STATE_IDLE,
	AUDIO_I2S_STATE_STARTED,
};

static enum audio_i2s_state state = AUDIO_I2S_STATE_UNINIT;

PINCTRL_DT_DEFINE(I2S_NL);

static nrfx_i2s_t i2s_inst = NRFX_I2S_INSTANCE(0);

static nrfx_i2s_config_t cfg = {
	/* Pins are configured by pinctrl. */
	.skip_gpio_cfg = true,
	.skip_psel_cfg = true,
	.irq_priority = DT_IRQ(I2S_NL, priority),
	.mode = NRF_I2S_MODE_MASTER,
	.format = NRF_I2S_FORMAT_I2S,
	.alignment = NRF_I2S_ALIGN_LEFT,
	.ratio = NRF_I2S_RATIO_256X,
	.mck_setup = NRF_I2S_MCK_32MDIV8, // enable_bypass should disable this
	.sample_width = NRF_I2S_SWIDTH_16BIT,
	.channels = NRF_I2S_CHANNELS_STEREO,
	.clksrc = NRF_I2S_CLKSRC_ACLK,
	.enable_bypass = true,
};

static i2s_blk_comp_callback_t i2s_blk_comp_callback;

static void i2s_comp_handler(nrfx_i2s_buffers_t const *released_bufs, uint32_t status)
{
	uint32_t frame_start_ts = audio_sync_timer_capture_get();

	if ((status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) && released_bufs &&
	    i2s_blk_comp_callback && (released_bufs->p_rx_buffer || released_bufs->p_tx_buffer)) {
		i2s_blk_comp_callback(frame_start_ts, released_bufs->p_rx_buffer,
				      released_bufs->p_tx_buffer);
	}
}


void audio_i2s_set_next_buf(const uint8_t *tx_buf)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_STARTED);
    __ASSERT_NO_MSG(tx_buf != NULL);

	const nrfx_i2s_buffers_t i2s_buf = {.p_rx_buffer = NULL,
					    .p_tx_buffer = (uint32_t *)tx_buf,
					    .buffer_size = I2S_SAMPLES_NUM};

	nrfx_err_t ret;

	ret = nrfx_i2s_next_buffers_set(&i2s_inst, &i2s_buf);
	__ASSERT_NO_MSG(ret == NRFX_SUCCESS);
}

void audio_i2s_start(const uint8_t *tx_buf)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_IDLE);
	__ASSERT_NO_MSG(tx_buf != NULL);

	const nrfx_i2s_buffers_t i2s_buf = {.p_rx_buffer = NULL,
					    .p_tx_buffer = (uint32_t *)tx_buf,
					    .buffer_size = I2S_SAMPLES_NUM};

	nrfx_err_t ret;

	/* Buffer size in 32-bit words */
	ret = nrfx_i2s_start(&i2s_inst, &i2s_buf, 0);
	__ASSERT_NO_MSG(ret == NRFX_SUCCESS);

	state = AUDIO_I2S_STATE_STARTED;
}

void audio_i2s_stop(void)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_STARTED);

	nrfx_i2s_stop(&i2s_inst);

	state = AUDIO_I2S_STATE_IDLE;
}

void audio_i2s_blk_comp_cb_register(i2s_blk_comp_callback_t blk_comp_callback)
{
	i2s_blk_comp_callback = blk_comp_callback;
}


void audio_i2s_init(void)
{
	__ASSERT_NO_MSG(state == AUDIO_I2S_STATE_UNINIT);

	nrfx_err_t ret;

	nrfx_clock_hfclkaudio_config_set(HFCLKAUDIO_12_288_MHZ);

	NRF_CLOCK->TASKS_HFCLKAUDIOSTART = 1;

	/* Wait for ACLK to start */
	while (!NRF_CLOCK_EVENT_HFCLKAUDIOSTARTED) {
		k_sleep(K_MSEC(1));
	}

	ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(I2S_NL), PINCTRL_STATE_DEFAULT);
	__ASSERT_NO_MSG(ret == 0);

	IRQ_CONNECT(DT_IRQN(I2S_NL), DT_IRQ(I2S_NL, priority), nrfx_isr, nrfx_i2s_0_irq_handler, 0);
	irq_enable(DT_IRQN(I2S_NL));

	ret = nrfx_i2s_init(&i2s_inst, &cfg, i2s_comp_handler);
	__ASSERT_NO_MSG(ret == NRFX_SUCCESS);

	state = AUDIO_I2S_STATE_IDLE;
}
