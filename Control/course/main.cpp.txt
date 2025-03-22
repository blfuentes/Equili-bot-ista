#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

// Logger tag for ESP-IDF logging
static const char *TAG = "app_main";

extern "C" void app_main();

void app_main(void)
{
    for (;;)
    {
        ESP_LOGI(TAG, "Hello from app_main!");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
