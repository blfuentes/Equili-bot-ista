#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <esp_log.h>
#include <esp_err.h>
#include <memory>

#include <MotorControl.h>
#include <bmi160_wrapper.h>
#include <PIDService.h>

// Logger tag for ESP-IDF logging
static const char *MAIN_TAG = "app_control_main";

enum class ModeType : u_int8_t {
    STANDING = 0,
    WAR_MODE = 1
};

enum class ParamType : u_int8_t {
    P = 0,
    I = 1,
    D = 2
};

// Led
constexpr gpio_num_t LED_PIN = GPIO_NUM_4;
PinGPIODefinition led;

// Push buttons
constexpr gpio_num_t MODE_PIN = GPIO_NUM_3;
auto current_mode = std::make_unique<ModeType>(ModeType::STANDING);
PinGPIODefinition mode;

constexpr gpio_num_t PARAM_PIN = GPIO_NUM_2;
auto current_param = std::make_unique<ParamType>(ParamType::P);
PinGPIODefinition param;

// Control
constexpr gpio_num_t CONTROL_PIN = GPIO_NUM_2;
#define POTENTIOMETER_ADC_CHANNEL   ADC1_CHANNEL_0  // GPIO1 on ESP32-C3
#define ADC_WIDTH_BIT               ADC_WIDTH_BIT_12  // 12-bit resolution (0-4095)
#define ADC_ATTEN_DB               ADC_ATTEN_DB_11   // Full-scale voltage ~3.3V

//

extern "C" void app_main();

// Mode helpers
void next_mode(std::unique_ptr<ModeType>& p_current_mode) {
    *p_current_mode = static_cast<ModeType>((static_cast<u_int8_t>(*p_current_mode) + 1) % 2);
}
constexpr std::array<const char*, 2> ModeStrings = { "Standing", "War mode" };

constexpr const char* mode_to_string(ModeType p_mode) {
    return ModeStrings.at(static_cast<size_t>(p_mode));
}

// Param helpers
void next_param(std::unique_ptr<ParamType>& p_current_param) {
    *p_current_param = static_cast<ParamType>((static_cast<u_int8_t>(*p_current_param) + 1) % 3);
}
constexpr std::array<const char*, 3> ParamStrings = { "P", "I", "D" };

constexpr const char* param_to_string(ParamType p_param) {
    return ParamStrings.at(static_cast<size_t>(p_param));
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Configuring LED PIN
    led = PinGPIODefinition(LED_PIN, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE);
    led.Configure();
    uint32_t led_value = 0;
    // Configuring MODE PIN
    mode = PinGPIODefinition(MODE_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN_DISABLE);
    mode.Configure();
    // Configuring PARAM PIN
    param = PinGPIODefinition(PARAM_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN_DISABLE);
    param.Configure();
    // // Configuring CONTROL PIN
    // // Configure ADC
    // adc1_config_width(ADC_WIDTH_BIT);
    // adc1_config_channel_atten(POTENTIOMETER_ADC_CHANNEL, ADC_ATTEN_DB);

    // // Characterize ADC (for better accuracy)
    // esp_adc_cal_characteristics_t adc_chars;
    // esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB, ADC_WIDTH_BIT, 1100, &adc_chars);  // 1100mV default Vref


    
    for (;;)
    {
        // led based on current mode
        if (*current_mode == ModeType::STANDING) 
            led_value = 1;
        else {
            led_value = !led_value;
        }
        gpio_set_level(LED_PIN, led_value);

        // read buttons
        if (!gpio_get_level(MODE_PIN)) {
            next_mode(current_mode);
        }
        if (!gpio_get_level(PARAM_PIN)){
            next_param(current_param);
        }
        ESP_LOGI(MAIN_TAG, "Current Mode: %s - Current Param: %s.", mode_to_string(*current_mode), param_to_string(*current_param));

        // vTaskDelay(pdMS_TO_TICKS(100));
        // if (param_value == 1) {
        //     printf("Param is pressed...\n");
        // } else {
        //     printf("Param is released...\n");
        // }

        // // test control pin
        // // Read raw ADC value (0-4095)
        // int raw_value = adc1_get_raw(POTENTIOMETER_ADC_CHANNEL);

        // // Convert to voltage (mV)
        // int voltage_mV = esp_adc_cal_raw_to_voltage(raw_value, &adc_chars);

        // printf("Raw ADC: %d, Voltage: %dmV\n", raw_value, voltage_mV);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
