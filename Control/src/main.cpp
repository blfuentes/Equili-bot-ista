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

#include "ControlStatus.h"
#include "PinDefinition.h"

// Logger tag for ESP-IDF logging
static const char *MAIN_TAG = "main_log";
static const char *ADC_TAG = "adc_log";
static const char *PARAMS_TAG = "params_log";

// Led
constexpr gpio_num_t LED_PIN = GPIO_NUM_4;
PinGPIODefinition led;

// Push buttons
constexpr gpio_num_t MODE_PIN = GPIO_NUM_3;
PinGPIODefinition mode;

constexpr gpio_num_t PARAM_PIN = GPIO_NUM_2;
PinGPIODefinition param;

// Control
constexpr gpio_num_t CONTROL_PIN = GPIO_NUM_0;
#define POT_ADC_CHANNEL     ADC_CHANNEL_0  // GPIO36 if using ESP32-WROOM
#define ADC_WIDTH           ADC_WIDTH_BIT_12
#define ADC_ATTEN          ADC_ATTEN_DB_12  // 0-3.1V range

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

// Status
ControlStatus current_status;

extern "C" void app_main();

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    //
    current_status = ControlStatus(ModeType::STANDING, ParamType::P);

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
    
    int raw_value;
    int voltage_value;

    ESP_LOGI(MAIN_TAG, "Current Mode: %s - Current Param: %s.", current_status.ModeToString(), current_status.ParamToString());
    ESP_LOGI(ADC_TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, POT_ADC_CHANNEL, current_status.adc_raw);
    ESP_LOGI(ADC_TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, POT_ADC_CHANNEL, current_status.voltage);

    for (;;)
    {
        // led based on current mode
        if (current_status.current_mode == ModeType::STANDING) 
            led_value = 1;
        else {
            led_value = !led_value;
        }
        gpio_set_level(LED_PIN, led_value);

        // read buttons
        if (!gpio_get_level(MODE_PIN)) {
            current_status.NextMode();
        }
        if (!gpio_get_level(PARAM_PIN)){
            current_status.NextParam();
        }
        
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, POT_ADC_CHANNEL, &raw_value));
        current_status.SetRaw(raw_value);
        
        if (do_calibration1_chan0) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, current_status.adc_raw, &voltage_value));
            current_status.SetVoltage(voltage_value);
        }
        
        if (current_status.HasChanged())
        {
            ESP_LOGI(MAIN_TAG, "Current Mode: %s - Current Param: %s.", current_status.ModeToString(), current_status.ParamToString());
            ESP_LOGI(ADC_TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, POT_ADC_CHANNEL, current_status.adc_raw);
            ESP_LOGI(ADC_TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, POT_ADC_CHANNEL, current_status.voltage);
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