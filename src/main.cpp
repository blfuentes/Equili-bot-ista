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

    if (bmi_configure(&dev) != BMI160_OK)
    {
        ESP_LOGE(TAG, "Failed to configure bmi160 device");
    }
    else
    {

        bmi160_sensor_data_adjusted accel_data;
        bmi160_sensor_data_adjusted gyro_data;

        for (;;)
        {
            if (bmi160_get_sensor_data_adjusted(BMI160_ACCEL_SEL | BMI160_GYRO_SEL, &accel_data, &gyro_data, &dev, true) == BMI160_OK)
            {
                printf("Accl: %.2f %.2f %.2f\n", accel_data.x, accel_data.y, accel_data.z);
                printf("Gyro: %.2f %.2f %.2f\n", gyro_data.x, gyro_data.y, gyro_data.z);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
