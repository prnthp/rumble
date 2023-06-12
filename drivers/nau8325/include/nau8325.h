#ifndef _NAU8325_H_
#define _NAU8325_H_

#include <zephyr/types.h>
#include <zephyr/device.h>

int nau8325_read_reg(const struct device* dev, uint16_t reg_addr, uint16_t *value_ptr);
int nau8325_write_reg(const struct device* dev, uint16_t reg_addr, uint16_t value);

#endif
