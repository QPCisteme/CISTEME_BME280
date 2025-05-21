#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include "cisteme_bme280_regmap.h"

struct bme280_config {
    struct i2c_dt_spec i2c;
};

struct bme280_data {
    /* Compensation parameters for BME280 */
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
    int32_t  t_fine;

    uint8_t ctrl_meas_val;
    uint8_t ctrl_hum_val;
};

// Déclaration des typedef
typedef int (*bme280_read_reg_t)(const struct device *dev, uint8_t addr, uint8_t *val);
typedef int (*bme280_write_reg_t)(const struct device *dev, uint8_t addr, uint8_t val);
typedef int (*bme280_read_meas_t)(const struct device *dev, int32_t *temp, uint32_t *hum, uint32_t *press);
typedef int (*bme280_cmd_t)(const struct device *dev);


// Déclaration de l'API
__subsystem struct bme280_driver_api {
    bme280_read_reg_t bme280_read_reg;
    bme280_write_reg_t bme280_write_reg;
    bme280_read_meas_t bme280_read_meas;
    bme280_cmd_t bme280_force_meas;
};

__syscall int bme280_read_reg(const struct device *dev, uint8_t addr, uint8_t *val);

static inline int z_impl_bme280_read_reg(const struct device *dev, uint8_t addr, uint8_t *val)
{
    const struct bme280_driver_api *api = (const struct bme280_driver_api *)dev->api;
    if (api->bme280_read_reg == NULL) {
        return -ENOSYS;
    }
    return api->bme280_read_reg(dev, addr, val);
}

__syscall int bme280_write_reg(const struct device *dev, uint8_t addr, uint8_t val);

static inline int z_impl_bme280_write_reg(const struct device *dev, uint8_t addr, uint8_t val)
{
    const struct bme280_driver_api *api = (const struct bme280_driver_api *)dev->api;
    if (api->bme280_write_reg == NULL) {
        return -ENOSYS;
    }
    return api->bme280_write_reg(dev, addr, val);
}

__syscall int bme280_read_meas(const struct device *dev, int32_t *temp, uint32_t *hum, uint32_t *press);

static inline int z_impl_bme280_read_meas(const struct device *dev, int32_t *temp, uint32_t *hum, uint32_t *press)
{
    const struct bme280_driver_api *api = (const struct bme280_driver_api *)dev->api;
    if (api->bme280_read_meas == NULL) {
        return -ENOSYS;
    }
    return api->bme280_read_meas(dev, temp, hum, press);
}

__syscall int bme280_force_meas(const struct device *dev);

static inline int z_impl_bme280_force_meas(const struct device *dev)
{
    const struct bme280_driver_api *api = (const struct bme280_driver_api *)dev->api;
    if (api->bme280_force_meas == NULL) {
        return -ENOSYS;
    }
    return api->bme280_force_meas(dev);
}

#include <syscalls/cisteme_bme280.h>