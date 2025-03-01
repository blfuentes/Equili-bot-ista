#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>

#include <MotorControl.h>

// Logger tag for ESP-IDF logging
static const char * MAIN_TAG = "app_main";

extern "C" void app_main();

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (false)
    {
        ESP_LOGE(MAIN_TAG, "Failed to initialize bmi160 device");
    }
    else
    {
        for(;;)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
