#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_log.h>

// Logger tag for ESP-IDF logging
static const char * MAIN_TAG = "app_main";

extern "C" void app_main();

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(100)); // Reduced delay for more responsive readings
    }
}