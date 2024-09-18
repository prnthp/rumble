
#include "audio_usb.h"

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_audio.h>
#include <data_fifo.h>

#include "configs.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_usb, LOG_LEVEL_INF);

#define USB_FRAME_SIZE_STEREO                                                                      \
	(((CONFIG_AUDIO_SAMPLE_RATE_HZ * CONFIG_AUDIO_BIT_DEPTH_OCTETS) / 1000) * 2)

static struct data_fifo *fifo_rx;

NET_BUF_POOL_FIXED_DEFINE(pool_out, CONFIG_FIFO_FRAME_SPLIT_NUM, USB_FRAME_SIZE_STEREO, 8,
			  net_buf_destroy);

static uint32_t rx_num_overruns;
static bool rx_first_data;

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

static void data_received(const struct device *dev, struct net_buf *buffer, size_t size)
{
	int ret;
	void *data_in;

	if (fifo_rx == NULL) {
		/* Throwing away data */
		net_buf_unref(buffer);
		return;
	}

	if (buffer == NULL || size == 0 || buffer->data == NULL) {
		/* This should never happen */
		ERR_CHK(-EINVAL);
	}

	/* Receive data from USB */
	if (size != USB_FRAME_SIZE_STEREO) {
		LOG_WRN("Wrong length: %d", size);
		net_buf_unref(buffer);
		return;
	}

	ret = data_fifo_pointer_first_vacant_get(fifo_rx, &data_in, K_NO_WAIT);

    /* RX FIFO can fill up due to retransmissions or disconnect */
	if (ret == -ENOMEM) {
		void *temp;
		size_t temp_size;

		rx_num_overruns++;
		if ((rx_num_overruns % 100) == 1) {
			LOG_WRN("USB RX overrun. Num: %d", rx_num_overruns);
		}

		ret = data_fifo_pointer_last_filled_get(fifo_rx, &temp, &temp_size, K_NO_WAIT);
		ERR_CHK(ret);

		data_fifo_block_free(fifo_rx, temp);

		ret = data_fifo_pointer_first_vacant_get(fifo_rx, &data_in, K_NO_WAIT);
	}

	ERR_CHK_MSG(ret, "RX failed to get block");

	memcpy(data_in, buffer->data, size);

	ret = data_fifo_block_lock(fifo_rx, &data_in, size);
	ERR_CHK_MSG(ret, "Failed to lock block");

	net_buf_unref(buffer);

	if (!rx_first_data) {
		LOG_INF("USB RX first data received.");
		rx_first_data = true;
	}
}

static const struct usb_audio_ops hp_ops = {
	.data_received_cb = data_received,
	.feature_update_cb = feature_update,
};

int audio_usb_start(struct data_fifo *fifo_rx_in)
{
	if (fifo_rx_in == NULL) {
		return -EINVAL;
	}

	fifo_rx = fifo_rx_in;

	return 0;
}

void audio_usb_stop(void)
{
	rx_first_data = false;
	fifo_rx = NULL;
}

int audio_usb_init(void)
{
    int ret;

	const struct device *const hp_dev = DEVICE_DT_GET_ONE(usb_audio_hp);

    if (!device_is_ready(hp_dev)) {
		LOG_ERR("Device USB Headphones is not ready");
		return -EIO;
    }

    usb_audio_register(hp_dev, &hp_ops);

    ret = usb_enable(NULL);
    
    if (ret) {
		LOG_ERR("Failed to enable USB");
		return ret;
    }
    
	LOG_INF("USB Audio initialze");

    return 0;
}