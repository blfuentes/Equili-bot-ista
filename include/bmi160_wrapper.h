#ifndef __BMI160_WRAPPER_H__
#define __BMI160_WRAPPER_H__

#include <stdint.h>
#include <bmi160.h>

bmi160_dev bmi_init_spi();
int8_t bmi_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
void bmi_delay(uint32_t period);

#endif //__BMI160_WRAPPER_H__
