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

#include "ssd1306.h"
#include "font8x8_basic.h"

#include "ControlStatus.h"
#include "PinDefinition.h"
#include <string.h>

// Logger tag for ESP-IDF logging
static const char *MAIN_TAG = "main_log";
static const char *ADC_TAG = "adc_log";
static const char *PARAMS_TAG = "params_log";
static const char *SCREEN_TAG = "SSD1306";

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

// Screen
SSD1306_t dev;
int center, top, bottom;
char display_buffer[16] = {0};  // Single reusable buffer

// PID defaults
constexpr int DEFAULT_P = 100;
constexpr int DEFAULT_I = 10;
constexpr int DEFAULT_D = 1;

void update_display() {
    ssd1306_clear_screen(&dev, false);

    // Line 0: Mode - Param
    memset(display_buffer, 0, sizeof(display_buffer));
    snprintf(display_buffer, sizeof(display_buffer), "%s - %s", 
             current_status.ModeToString(), current_status.ParamToString());
    ssd1306_display_text(&dev, 0, display_buffer, 16, false);

    // // Line 1: Raw ADC
    // memset(display_buffer, 0, sizeof(display_buffer));
    // snprintf(display_buffer, sizeof(display_buffer), "Raw: %d", current_status.adc_raw);
    // ssd1306_display_text(&dev, 1, display_buffer, 16, false);

    // // Line 2: Voltage
    // memset(display_buffer, 0, sizeof(display_buffer));
    // snprintf(display_buffer, sizeof(display_buffer), "Voltage: %d", current_status.voltage);
    // ssd1306_display_text(&dev, 2, display_buffer, 16, false);

    // // Line 3: PID Values
    // memset(display_buffer, 0, sizeof(display_buffer));
    // snprintf(display_buffer, sizeof(display_buffer), "P:%d I:%d D:%d", 100, 10, 1);
    // ssd1306_display_text(&dev, 3, display_buffer, 16, false);

    // Line 1: PID Values
    memset(display_buffer, 0, sizeof(display_buffer));
    snprintf(display_buffer, sizeof(display_buffer), "P: %d", current_status.current_P);
    ssd1306_display_text(&dev, 1, display_buffer, 16, false);

    // Line 2: PID Values
    memset(display_buffer, 0, sizeof(display_buffer));
    snprintf(display_buffer, sizeof(display_buffer), "I: %d", current_status.current_I);
    ssd1306_display_text(&dev, 2, display_buffer, 16, false);

    // Line 3: PID Values
    memset(display_buffer, 0, sizeof(display_buffer));
    snprintf(display_buffer, sizeof(display_buffer), "D: %.1f", current_status.current_D);
    ssd1306_display_text(&dev, 3, display_buffer, 16, false);
}

void update_pid(int orientation)
{
    switch(current_status.current_param)
    {
        case ParamType::P:
            current_status.current_P += orientation == 1 ? 10 : -10;
            break;
        case ParamType::I:
            current_status.current_I += orientation == 1 ? 1 : -1;
            break;
        case ParamType::D:
            current_status.current_D += orientation == 1 ? .1 : -.1;
            break;
    }
}

extern "C" void app_main();

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    // OLED
    ESP_LOGI(SCREEN_TAG, "INTERFACE is i2c");
    ESP_LOGI(SCREEN_TAG, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
    ESP_LOGI(SCREEN_TAG, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
    ESP_LOGI(SCREEN_TAG, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

    ESP_LOGI(SCREEN_TAG, "Panel is 128x32");
    ssd1306_init(&dev, 128, 32);
    ssd1306_clear_screen(&dev, false);

    ssd1306_contrast(&dev, 0xff);

    ssd1306_display_text(&dev, 0, "STARTING", 9, false);

    //
    current_status = ControlStatus(DEFAULT_P, DEFAULT_I, DEFAULT_D, ModeType::STANDING, ParamType::P);

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
    
    int pid_orientation = 0;
    int raw_value;
    int voltage_value;

    ESP_LOGI(MAIN_TAG, "Current Mode: %s - Current Param: %s.", current_status.ModeToString(), current_status.ParamToString());
    ESP_LOGI(ADC_TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, POT_ADC_CHANNEL, current_status.adc_raw);
    ESP_LOGI(ADC_TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, POT_ADC_CHANNEL, current_status.voltage);

    update_display();

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
        
        pid_orientation = current_status.ControlChanged();
        if (pid_orientation != 0)
        {
            update_pid(pid_orientation);
        }

        if (current_status.HasChanged())
        {
            ESP_LOGI(MAIN_TAG, "Current Mode: %s - Current Param: %s.", current_status.ModeToString(), current_status.ParamToString());
            ESP_LOGI(ADC_TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, POT_ADC_CHANNEL, current_status.adc_raw);
            ESP_LOGI(ADC_TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, POT_ADC_CHANNEL, current_status.voltage);

            update_display();
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