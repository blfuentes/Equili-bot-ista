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
                              struct bmi160_sensor_data_adjusted *accel,
                              struct bmi160_sensor_data_adjusted *gyro,
                              const struct bmi160_dev *dev,
                              bool adjusted);

struct bmi160_sensor_data_adjusted
{
    /*! X-axis sensor data */
    float x;

    /*! Y-axis sensor data */
    float y;

    /*! Z-axis sensor data */
    float z;

    /*! sensor time */
    uint32_t sensortime;
};

#endif //__BMI160_WRAPPER_H__
