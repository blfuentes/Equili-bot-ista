#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>


extern "C" void app_main();

void app_main(void)
{
	while(1)
    {   
		printf("Hello from app_main!\n");
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
