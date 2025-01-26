#ifndef __BMI160_WRAPPER_H__
#define __BMI160_WRAPPER_H__

#include <stdint.h>
#include <bmi160.h>

bmi160_dev bmi_init_spi();
int8_t bmi_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
void bmi_delay(uint32_t period);

int8_t bmi_configure(bmi160_dev *dev);
int8_t bmi160_get_sensor_data_adjusted(uint8_t select_sensor,
                              struct bmi160_sensor_data_wrapper *accel,
                              struct bmi160_sensor_data_wrapper *gyro,
                              const struct bmi160_dev *dev,
                              bool adjusted);

struct bmi16_sensor_data_adj
{
    float x;

    float y;

    float z;

    int sensortime;
};

struct bmi160_sensor_data_wrapper
{
    bmi160_sensor_data raw_data;
    bmi16_sensor_data_adj adj_data;
};

#endif //__BMI160_WRAPPER_H__
