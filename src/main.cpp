#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <bmi160_wrapper.h>

// Logger tag for ESP-IDF logging
static const char *TAG = "app_main";

extern "C" void app_main();

void app_main(void)
{
    bmi160_dev dev = bmi_init_spi();

    int8_t rslt;

    rslt = bmi160_init(&dev);

    if (rslt == BMI160_OK)
    {
        printf("BMI160 initialization success !\n");
        printf("Chip ID 0x%X\n", dev.chip_id);
    }
    else
    {
        printf("BMI160 initialization failure !\n");
    }

    /* Select the Output data rate, range of accelerometer sensor */
    dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&dev);

    bmi160_sensor_data accel_data;
    bmi160_sensor_data gyro_data;

    float accToG = 16.0f / float((1 << 15) - 1);
    float gyroToDps = 2000.0f / float((1 << 15) - 1);
    for (;;)
    {
        if (bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL, &accel_data, &gyro_data, &dev) == BMI160_OK)
        {
            printf("Accl: %.2f %.2f %.2f\n", accel_data.x * accToG, accel_data.y * accToG, accel_data.z * accToG);
            printf("Gyro: %.2f %.2f %.2f\n", gyro_data.x * gyroToDps, gyro_data.y * gyroToDps, gyro_data.z * gyroToDps);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
