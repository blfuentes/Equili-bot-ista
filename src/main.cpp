#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>

#include <MotorControl.h>

// Logger tag for ESP-IDF logging
static const char * MAIN_TAG = "app_main";

// motor pins
constexpr gpio_num_t MOTOR_A_IN_1 = GPIO_NUM_18;
constexpr gpio_num_t MOTOR_A_IN_2 = GPIO_NUM_19;
constexpr gpio_num_t MOTOR_A_PWM = GPIO_NUM_23;

constexpr gpio_num_t MOTOR_B_IN_1 = GPIO_NUM_26;
constexpr gpio_num_t MOTOR_B_IN_2 = GPIO_NUM_27;
constexpr gpio_num_t MOTOR_B_PWM = GPIO_NUM_5;

constexpr gpio_num_t STBY = GPIO_NUM_33;

constexpr ledc_mode_t LEDC_SPEED_MODE = LEDC_LOW_SPEED_MODE;

MotorDefinition leftMotor;
MotorDefinition rightMotor;
PinGPIODefinition stby;

extern "C" void app_main();

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    leftMotor = MotorDefinition(MOTOR_A_IN_1, MOTOR_A_IN_2, 0, 1, MOTOR_A_PWM, LEDC_CHANNEL_0, LEDC_SPEED_MODE, LEDC_TIMER_0);
    rightMotor = MotorDefinition(MOTOR_B_IN_1, MOTOR_B_IN_2, 1, 0, MOTOR_B_PWM, LEDC_CHANNEL_1, LEDC_SPEED_MODE, LEDC_TIMER_1);

    stby = PinGPIODefinition(STBY, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE);

    leftMotor.Configure();
    rightMotor.Configure();
    stby.Configure();

    gpio_set_level(STBY, 1);

    if (false)
    {
        ESP_LOGE(MAIN_TAG, "Failed to initialize bmi160 device");
    }
    else
    {
        for(;;)
        {
            leftMotor.Drive(100);
            vTaskDelay(pdMS_TO_TICKS(200));
            rightMotor.Drive(100);

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
