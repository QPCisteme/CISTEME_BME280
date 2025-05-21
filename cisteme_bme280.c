#define DT_DRV_COMPAT cisteme_bme280

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>

#include <cisteme_bme280.h>
#include "cisteme_bme280_regmap.h"

/* Fonctions de compensation pour le BME280 */

static int32_t bme280_compensate_temp(int32_t adc_T, struct bme280_data *data)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)data->dig_T1 << 1))) * ((int32_t)data->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)data->dig_T1)) * ((adc_T >> 4) - ((int32_t)data->dig_T1))) >> 12) *
            ((int32_t)data->dig_T3)) >> 14;
    data->t_fine = var1 + var2;
    T = (data->t_fine * 5 + 128) >> 8;
    return T; // en 0.01°C
}

static uint32_t bme280_compensate_press(int32_t adc_P, struct bme280_data *data)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)data->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)data->dig_P6;
    var2 = var2 + ((var1 * (int64_t)data->dig_P5) << 17);
    var2 = var2 + (((int64_t)data->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)data->dig_P3) >> 8) + ((var1 * (int64_t)data->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)data->dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // éviter la division par zéro
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)data->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)data->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)data->dig_P7) << 4);
    return (uint32_t)p; // en Pa (Q24.8 format)
}

static uint32_t bme280_compensate_hum(int32_t adc_H, struct bme280_data *data)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (data->t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)data->dig_H4) << 20) -
                    (((int32_t)data->dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)data->dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)data->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)data->dig_H2) + 8192) >> 14));
    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)data->dig_H1)) >> 4);
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12); // en 0.001 %RH
}

static int read_reg(const struct device *dev, uint8_t addr, uint8_t *val)
{
    const struct bme280_config *config = dev->config;
    int ret;

    ret = i2c_write_read_dt(&config->i2c, &addr, 1, val, 1);

    return ret;
}

static int write_reg(const struct device *dev, uint8_t addr, uint8_t val)
{
    const struct bme280_config *config = dev->config;
    int ret;

    uint8_t buf[2]={addr, val};

    ret = i2c_write_dt(&config->i2c, buf, 2);

    return ret;
}

static int read_meas(const struct device *dev, int32_t *temp, uint32_t *hum, uint32_t *press)
{
    const struct bme280_config *config = dev->config;
    int ret;

    uint8_t buf[8];

    ret = i2c_burst_read_dt(&config->i2c, BME280_PRESS_MSB, buf, 8);

    // buf[0..2]: pressure, buf[3..5]: temperature, buf[6..7]: humidity
    uint32_t adc_P = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | ((uint32_t)buf[2] >> 4);
    uint32_t adc_T = ((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | ((uint32_t)buf[5] >> 4);
    uint32_t adc_H = ((uint32_t)buf[6] << 8) | (uint32_t)buf[7];

    struct bme280_data *data = dev->data;

    *temp = bme280_compensate_temp(adc_T, data);
    *press = bme280_compensate_press(adc_P, data);
    *hum = bme280_compensate_hum(adc_H, data);

    return ret;
}

static int force_meas(const struct device *dev)
{
    struct bme280_data *data = dev->data;
    bme280_write_reg(dev, BME280_CTRL_MEAS, (data->ctrl_meas_val)|0x01);
    return 0;    
}

static const struct bme280_driver_api bme280_api = {
    .bme280_read_reg = &read_reg,
    .bme280_write_reg = &write_reg,
    .bme280_read_meas = &read_meas,
    .bme280_force_meas = &force_meas,
};

static struct bme280_data data;

static int bme280_init(const struct device *dev)
{
    const struct bme280_config *config = dev->config;

    if (!device_is_ready(config->i2c.bus)) {
        return -ENODEV;
    }

    struct bme280_data *data = dev->data;
    uint8_t buf[26];
    int ret;

    // Lire les registres de compensation température et pression (0x88 à 0xA1)
    i2c_burst_read_dt(&config->i2c, BME280_CALIB_LSB, buf, 26);

    data->dig_T1 = sys_le16_to_cpu(*(uint16_t *)&buf[0]);
    data->dig_T2 = sys_le16_to_cpu(*(int16_t *)&buf[2]);
    data->dig_T3 = sys_le16_to_cpu(*(int16_t *)&buf[4]);
    data->dig_P1 = sys_le16_to_cpu(*(uint16_t *)&buf[6]);
    data->dig_P2 = sys_le16_to_cpu(*(int16_t *)&buf[8]);
    data->dig_P3 = sys_le16_to_cpu(*(int16_t *)&buf[10]);
    data->dig_P4 = sys_le16_to_cpu(*(int16_t *)&buf[12]);
    data->dig_P5 = sys_le16_to_cpu(*(int16_t *)&buf[14]);
    data->dig_P6 = sys_le16_to_cpu(*(int16_t *)&buf[16]);
    data->dig_P7 = sys_le16_to_cpu(*(int16_t *)&buf[18]);
    data->dig_P8 = sys_le16_to_cpu(*(int16_t *)&buf[20]);
    data->dig_P9 = sys_le16_to_cpu(*(int16_t *)&buf[22]);
    data->dig_H1 = buf[25];

    // Lire les registres de compensation humidité (0xE1 à 0xE7)
    i2c_burst_read_dt(&config->i2c, BME280_CALIB_MSB, buf, 7);

    data->dig_H2 = sys_le16_to_cpu(*(int16_t *)&buf[0]);
    data->dig_H3 = buf[2];
    data->dig_H4 = (int16_t)((buf[3] << 4) | (buf[4] & 0x0F));
    data->dig_H5 = (int16_t)((buf[5] << 4) | (buf[4] >> 4));
    data->dig_H6 = (int8_t)buf[6];

    data->ctrl_meas_val=0x24;
    data->ctrl_hum_val=0x01;

    i2c_write_dt(&config->i2c, BME280_CTRL_MEAS, data->ctrl_meas_val);
    i2c_write_dt(&config->i2c, BME280_CTRL_HUM, data->ctrl_hum_val);

    return 0;
}

#define BME280_DEFINE(inst)                                      \
    static const struct bme280_config bme280_config_##inst = {   \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                       \
    };                                                           \
    DEVICE_DT_INST_DEFINE(inst,                                  \
                          bme280_init,                           \
                          NULL,                                  \
                          &data,                                 \
                          &bme280_config_##inst,                 \
                          POST_KERNEL,                           \
                          90,                                    \
                          &bme280_api);

DT_INST_FOREACH_STATUS_OKAY(BME280_DEFINE)