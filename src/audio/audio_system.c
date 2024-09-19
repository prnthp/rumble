
#include <zephyr/kernel.h>
#include <data_fifo.h>

#include "audio_usb.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int audio_system_init(void)
{
    int ret;
    ret = audio_usb_init();
    if (ret)
    {
        LOG_ERR("Failed to initialize USB: %d", ret);
    }
}