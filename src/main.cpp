#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <bmi160_wrapper.h>
#include <servo.h>

// Logger tag for ESP-IDF logging
static const char * MAIN_TAG = "app_main";

extern "C" void app_main();

Bmi160<Bmi160SpiConfig> imu;
Servo myServo;

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    myServo.initHw();
    uint32_t angle = 50;
    bool increasing = false;

    for(;;)
    {
        // Set the servo position
        myServo.setPos(angle);

        // Update the angle
        if (increasing) {
            angle++;
            if (angle >= 180) {
                increasing = false; // Reverse direction
            }
        } else {
            angle--;
            if (angle <= 0) {
                increasing = true; // Reverse direction
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return;

    Bmi160SpiConfig config = {
        .spidev = SPI3_HOST,
        .mosi = GPIO_NUM_23,
        .miso = GPIO_NUM_19,
        .sclk = GPIO_NUM_18,
        .cs   = GPIO_NUM_21,
        .speed= 4*1000*1000
    };

    if (imu.init(config) != BMI160_OK)
    {
        ESP_LOGE(MAIN_TAG, "Failed to initialize bmi160 device");
    }
    else
    {
        Bmi160Data accel_data, gyro_data;
        for(;;)
        {
            imu.getData(accel_data, gyro_data);

            printf("Raw-Accl: %6d %6d %6d\n", accel_data.raw_data.x, accel_data.raw_data.y, accel_data.raw_data.z);
            printf("Adj-Accl: %.2f %.2f %.2f\n", accel_data.adj_data.x, accel_data.adj_data.y, accel_data.adj_data.z);
            printf("Raw-Gyro: %6d %6d %6d\n", gyro_data.raw_data.x, gyro_data.raw_data.y, gyro_data.raw_data.z);
            printf("Adj-Gyro: %.2f %.2f %.2f\n", gyro_data.adj_data.x, gyro_data.adj_data.y, gyro_data.adj_data.z);

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
