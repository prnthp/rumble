#ifndef _CONFIGS_H_
#define _CONFIGS_H_

#define CONFIG_AUDIO_BIT_DEPTH_OCTETS 2 // 2 = 16-bit
#define CONFIG_AUDIO_SAMPLE_RATE_HZ 48000 // Hz
#define CONFIG_FIFO_FRAME_SPLIT_NUM 10 // ms
#define CONFIG_AUDIO_BIT_DEPTH_BITS 16
#define CONFIG_I2S_LRCK_FREQ_HZ CONFIG_AUDIO_SAMPLE_RATE_HZ
#define CONFIG_I2S_CH_NUM 2
#define CONFIG_AUDIO_MAX_PRES_DLY_US 60000 // us

#endif /* _CONFIGS_H_ */
