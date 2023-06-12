#include "nau8325.h"

#include <zephyr/types.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/byteorder.h>

#define LOG_LEVEL CONFIG_NAU8325_LOG_LEVEL
#define DT_DRV_COMPAT nuvoton_nau8325

LOG_MODULE_REGISTER(NAU8325, LOG_LEVEL);

uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_FAST);

struct nau8325_dev_data_t {
    uint16_t data;
};

struct nau8325_dev_cfg_t {
    struct i2c_dt_spec i2c;
};

static int nau8325_init(const struct device* dev) {
    const struct nau8325_dev_cfg_t* const i2c = dev->config;

    if (!i2c_is_ready_dt(&i2c->i2c)) {
        LOG_ERR("I2C bus not ready!");
        return -ENODEV;
    }

    int result = i2c_configure(i2c->i2c.bus, I2C_SPEED_SET(I2C_SPEED_STANDARD));

    LOG_INF("Initialized NAU8325 %d", result);

    return 0;
}

int nau8325_read_reg(const struct device* dev, uint16_t reg_addr, uint16_t *value_ptr) {
    const struct nau8325_dev_cfg_t* config = dev->config;
    uint8_t read_buf[2];
    uint8_t write_buf[2];
    int ret;

    sys_put_be16(reg_addr, &write_buf[0]);

    ret = i2c_write_read_dt(&config->i2c, write_buf, sizeof(write_buf), read_buf, sizeof(read_buf));
    if (ret < 0) {
        LOG_ERR("Unable to read register %d", reg_addr);
        return ret;
    }

    *value_ptr = (read_buf[0] << 8) | read_buf[1];

    return 0;
}

int nau8325_write_reg(const struct device* dev, uint16_t reg_addr, uint16_t value) {
    const struct nau8325_dev_cfg_t* config = dev->config;
    uint8_t buf[4];

    sys_put_be16(reg_addr, &buf[0]);
    sys_put_be16(value, &buf[2]);

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

#define NAU8325_DEFINE(inst)                                        \
    static struct nau8325_dev_data_t nau8325_data_##inst = {        \
    };                                                              \
    static const struct nau8325_dev_cfg_t nau8325_cfg_##inst = {    \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                      \
    };                                                              \
    DEVICE_DT_INST_DEFINE(inst,                                     \
                          nau8325_init,                             \
                          NULL,                                     \
                          &nau8325_data_##inst,                     \
                          &nau8325_cfg_##inst,                      \
                          POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY, \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(NAU8325_DEFINE)

