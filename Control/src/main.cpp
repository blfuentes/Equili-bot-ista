#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <esp_log.h>
#include <esp_err.h>
#include <memory>

#include "PinDefinition.h"

// Logger tag for ESP-IDF logging
static const char *MAIN_TAG = "main_log";
static const char *ADC_TAG = "adc_log";
static const char *PARAMS_TAG = "params_log";

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
constexpr gpio_num_t CONTROL_PIN = GPIO_NUM_0;
#define POT_ADC_CHANNEL     ADC_CHANNEL_0  // GPIO36 if using ESP32-WROOM
#define ADC_WIDTH           ADC_WIDTH_BIT_12
#define ADC_ATTEN          ADC_ATTEN_DB_12  // 0-3.1V range

static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

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
    // Configure ADC
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_ADC_CHANNEL, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, POT_ADC_CHANNEL, ADC_ATTEN, &adc1_cali_chan0_handle);
    
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

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, POT_ADC_CHANNEL, &adc_raw[0][0]));
        ESP_LOGI(ADC_TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, POT_ADC_CHANNEL, adc_raw[0][0]);
        if (do_calibration1_chan0) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
            ESP_LOGI(ADC_TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, POT_ADC_CHANNEL, voltage[0][0]);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0)
    {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
}


/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;
    if (!calibrated) {
        ESP_LOGI(ADC_TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(ADC_TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(ADC_TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(ADC_TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
    ESP_LOGI(ADC_TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
}