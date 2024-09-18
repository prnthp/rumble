
#include "audio_datapath.h"

#include <zephyr/kernel.h>

#include "audio_i2s.h"
#include "audio_system.h"
#include "configs.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_datapath, LOG_LEVEL_INF);

#define BLK_PERIOD_US 1000

/* Total sample FIFO period in microseconds */
#define FIFO_SMPL_PERIOD_US (CONFIG_AUDIO_MAX_PRES_DLY_US * 2)
#define FIFO_NUM_BLKS	    NUM_BLKS(FIFO_SMPL_PERIOD_US)
#define MAX_FIFO_SIZE	    (FIFO_NUM_BLKS * BLK_SIZE_SAMPLES(CONFIG_AUDIO_SAMPLE_RATE_HZ) * 2)

int audio_datapath_init(void)
{
	memset(&ctrl_blk, 0, sizeof(ctrl_blk));
	audio_i2s_blk_comp_cb_register(audio_datapath_i2s_blk_complete);
	audio_i2s_init();
	ctrl_blk.datapath_initialized = true;
	ctrl_blk.drift_comp.enabled = true;
	ctrl_blk.pres_comp.enabled = true;

	ctrl_blk.pres_comp.pres_delay_us = CONFIG_BT_AUDIO_PRESENTATION_DELAY_US;

	return 0;
}