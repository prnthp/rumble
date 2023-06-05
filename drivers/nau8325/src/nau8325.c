#include "nau8325.h"

#include <zephyr/types.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>

#define LOG_LEVEL CONFIG_NAU8325_LOG_LEVEL
#define DT_DRV_COMPAT nuvoton_nau8325

LOG_MODULE_REGISTER(NAU8325, LOG_LEVEL);

struct nau8325_dev_data_t {
    uint16_t data;
};

struct nau8325_dev_cfg_t {
    struct i2c_dt_spec i2c_bus;
};

static int nau8325_init(const struct device* dev) {
    const struct nau8325_dev_cfg_t* const i2c = dev->config;

    if (!i2c_is_ready_dt(&i2c->i2c_bus)) {
        LOG_ERR("I2C bus not ready!");
        return -ENODEV;
    }

    return 0;
}

#define NAU8325_DEFINE(inst)                                        \
    static struct nau8325_dev_data_t nau8325_data_##inst = {        \
    };                                                              \
    static const struct nau8325_dev_cfg_t nau8325_cfg_##inst = {    \
        .i2c_bus = I2C_DT_SPEC_INST_GET(inst),                      \
    };                                                              \
    DEVICE_DT_INST_DEFINE(inst,                                     \
                          nau8325_init,                             \
                          NULL,                                     \
                          &nau8325_data_##inst,                     \
                          &nau8325_cfg_##inst,                      \
                          POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY, \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(NAU8325_DEFINE)

