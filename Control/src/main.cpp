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
static const char *MODE_TAG = "mode_log";
static const char *ADC_TAG = "adc_log";
static const char *BLOCK_TAG = "block_log";
static const char *SCREEN_TAG = "ssd1306_log";
static const char *JOYSTICK_TAG = "joystick_log";
static const char *REPORT_TAG = "report_log";

// Led
constexpr gpio_num_t LED_PIN = GPIO_NUM_4;
PinGPIODefinition led;
uint32_t led_value = 0;

// Push buttons
constexpr gpio_num_t MODE_PIN = GPIO_NUM_2;
PinGPIODefinition mode;

constexpr gpio_num_t BLOCK_PIN = GPIO_NUM_8;
PinGPIODefinition block;

// Control
constexpr gpio_num_t CONTROL_PIN = GPIO_NUM_3;
constexpr adc_channel_t POT_ADC_CHANNEL = ADC_CHANNEL_3; // GPIO03
constexpr adc_atten_t ADC_ATTEN = ADC_ATTEN_DB_12; // 0-3.1V range

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

// Status
ControlStatus current_status;

// Screen
SSD1306_t oled_dev;
int center, top, bottom;
char lock_display = {0}; // Single reusable buffer
char display_buffer[16] = {0};  // Single reusable buffer

// PID defaults
constexpr int DEFAULT_P = 100;
constexpr int DEFAULT_I = 10;
constexpr int DEFAULT_D = 1;

// JOYSTICK
constexpr adc_channel_t JOYSTICK_X = ADC_CHANNEL_0; // GPIO00
constexpr adc_channel_t JOYSTICK_Y = ADC_CHANNEL_1; // GPIO01
constexpr gpio_num_t JOYSTICK_BUTTON_PIN = GPIO_NUM_9;
PinGPIODefinition param;

void update_movement_display()
{
    // Line 3: Joystick Values
    memset(display_buffer, 0, sizeof(display_buffer));
    snprintf(display_buffer, sizeof(display_buffer), "X:%5d Y:%5d", current_status.current_X, current_status.current_Y);
    ssd1306_display_text(&oled_dev, 3, display_buffer, 16, false);
}

void update_display() {
    ssd1306_clear_screen(&oled_dev, false);

    // Line 0: Mode - Param
    memset(display_buffer, 0, sizeof(display_buffer));
    lock_display = current_status.current_lock == LockType::LOCKED ? 'L' : 'U';//static_cast<char>(0x1F512) : static_cast<char>(0x1F513);
    snprintf(display_buffer, sizeof(display_buffer), "%s - %s (%c)", 
             current_status.ModeToString(), current_status.ParamToString(), lock_display);
    ssd1306_display_text(&oled_dev, 0, display_buffer, 16, false);

   // Line 2: PID Headers
   memset(display_buffer, 0, sizeof(display_buffer));
   snprintf(display_buffer, sizeof(display_buffer), "%3s %3s %4s", "P", "I", "D");
   ssd1306_display_text(&oled_dev, 1, display_buffer, 16, false);

    // Line 2: PID Values
    memset(display_buffer, 0, sizeof(display_buffer));
    snprintf(display_buffer, sizeof(display_buffer), "%3d %3d %4.1f", current_status.current_P, current_status.current_I, current_status.current_D);
    ssd1306_display_text(&oled_dev, 2, display_buffer, 16, false);

    // Line 3: Joystick Values
    update_movement_display();
}

void update_pid(int orientation)
{
    switch(current_status.current_param)
    {
        case ParamType::P:
            current_status.current_P += orientation == 1 ? 10 : -10;
            if (current_status.current_P < 0) current_status.current_P = 0;
            break;
        case ParamType::I:
            current_status.current_I += orientation == 1 ? 1 : -1;
            if (current_status.current_I < 0) current_status.current_I = 0;
            break;
        case ParamType::D:
            current_status.current_D += orientation == 1 ? .1 : -.1;
            if (current_status.current_D < 0) current_status.current_D = 0;
            break;
    }
}

extern "C" void app_main();

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(MAIN_TAG, "Starting application...");

    // OLED
    ESP_LOGI(SCREEN_TAG, "Initializing SSD1306 OLED display...");
    ESP_LOGI(SCREEN_TAG, "INTERFACE is i2c");
    ESP_LOGI(SCREEN_TAG, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
    ESP_LOGI(SCREEN_TAG, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
    ESP_LOGI(SCREEN_TAG, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
    ESP_LOGI(SCREEN_TAG, "Panel is 128x32");

    i2c_master_init(&oled_dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

    ssd1306_init(&oled_dev, 128, 32);
    ssd1306_clear_screen(&oled_dev, false);
    ssd1306_contrast(&oled_dev, 0xff);
    char starting_text[] = "STARTING...";
    ssd1306_display_text(&oled_dev, 0, starting_text, 9, false);

    ESP_LOGI(SCREEN_TAG, "SSD1306 initialized");

    // GPIOs
    ESP_LOGI(MAIN_TAG, "Configuring GPIOs...");
    // Configuring LED PIN
    led = PinGPIODefinition(LED_PIN, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE);
    led.Configure();
    ESP_LOGI(MODE_TAG, "Led configured on GPIO %d", LED_PIN);

    // Configuring MODE PIN
    mode = PinGPIODefinition(MODE_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN_DISABLE);
    mode.Configure();
    ESP_LOGI(MODE_TAG, "Mode configured on GPIO %d", MODE_PIN);

    // Configuring LOCK PIN
    block = PinGPIODefinition(BLOCK_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN_DISABLE);
    block.Configure();
    ESP_LOGI(BLOCK_TAG, "Block configured on GPIO %d", BLOCK_PIN);

    ESP_LOGI(MAIN_TAG, "GPIOs configured");

    // Configure base ADC
    ESP_LOGI(ADC_TAG, "Configuring ADC...");
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Configure POT ADC
    adc_oneshot_chan_cfg_t pot_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_ADC_CHANNEL, &pot_config));
    adc_cali_handle_t adc1_cali_control_handle = NULL;
    bool do_calibration1_control = example_adc_calibration_init(ADC_UNIT_1, POT_ADC_CHANNEL, ADC_ATTEN, &adc1_cali_control_handle);
    
    int pid_orientation = 0;
    int raw_value;
    int voltage_value;

    // calculate median value for control
    int median_voltage = 0;
    for (int i = 0; i < 50; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, POT_ADC_CHANNEL, &raw_value));
        median_voltage += raw_value;
    }
    median_voltage /= 50;
    ESP_LOGI(ADC_TAG, "Median Value: %d", median_voltage);
    current_status = ControlStatus(DEFAULT_P, DEFAULT_I, DEFAULT_D, ModeType::STANDING, ParamType::P, LockType::UNLOCKED);
    current_status.SetRaw(median_voltage);
    if (do_calibration1_control) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_control_handle, current_status.adc_raw, &median_voltage));
        current_status.SetVoltage(voltage_value);
    }

    ESP_LOGI(ADC_TAG, "ADC for POT configured on GPIO %d", POT_ADC_CHANNEL);

    update_display();

    // JOYSTICK
    // Configure GPIO for joystick
    param = PinGPIODefinition(JOYSTICK_BUTTON_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN_DISABLE);
    param.Configure();
    ESP_LOGI(JOYSTICK_TAG, "Joystick button configured on GPIO %d", JOYSTICK_BUTTON_PIN);

    // Configure Joystick ADC
    ESP_LOGI(JOYSTICK_TAG, "Configuring ADC for Joystick...");
    int raw_value_joystick_x, raw_value_joystick_y;
    int voltage_value_joystick_x, voltage_value_joystick_y;

    adc_oneshot_chan_cfg_t config_joystick = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_X, &config_joystick));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_Y, &config_joystick));

    adc_cali_handle_t adc1_cali_joy_x_handle = NULL;
    bool do_calibration1_joy_x = example_adc_calibration_init(ADC_UNIT_1, JOYSTICK_X, ADC_ATTEN, &adc1_cali_joy_x_handle);
    adc_cali_handle_t adc1_cali_joy_y_handle = NULL;
    bool do_calibration1_joy_y = example_adc_calibration_init(ADC_UNIT_1, JOYSTICK_Y, ADC_ATTEN, &adc1_cali_joy_y_handle);    

    // calculate median value for joystick
    int median_value_joystick_x = 0;
    int median_value_joystick_y = 0;
    for (int i = 0; i < 50; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK_X, &raw_value_joystick_x));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK_Y, &raw_value_joystick_y));
        median_value_joystick_x += raw_value_joystick_x;
        median_value_joystick_y += raw_value_joystick_y;
    }
    median_value_joystick_x /= 50;
    median_value_joystick_y /= 50;
    current_status.default_X = median_value_joystick_x;
    current_status.default_Y = median_value_joystick_y;
    ESP_LOGI(JOYSTICK_TAG, "ADC for Joystick configured on GPIO %d and %d", JOYSTICK_X, JOYSTICK_Y);
    ESP_LOGI(JOYSTICK_TAG, "Median Value X: %d Y: %d", median_value_joystick_x, median_value_joystick_y);

    bool firstRun = true;
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
        if (!gpio_get_level(BLOCK_PIN)){
            current_status.NextLock();
        }
        if(!gpio_get_level(JOYSTICK_BUTTON_PIN)) {
            current_status.NextParam();
        }
        
        // Read Control values
        if (current_status.current_lock == LockType::UNLOCKED){
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, POT_ADC_CHANNEL, &raw_value));
            current_status.SetRaw(raw_value);
            
            if (do_calibration1_control) {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_control_handle, current_status.adc_raw, &voltage_value));
                current_status.SetVoltage(voltage_value);
            }
            
            pid_orientation = current_status.ControlChanged();
            if (pid_orientation != 0 && !firstRun)
            {
                update_pid(pid_orientation);
            }
            else
            {
                firstRun = false;
            }
        }

        // Read joystick values
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK_X, &raw_value_joystick_x));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, JOYSTICK_Y, &raw_value_joystick_y));
        if(do_calibration1_joy_x) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_joy_x_handle, raw_value_joystick_x, &voltage_value_joystick_x));
        }
        if(do_calibration1_joy_y) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_joy_y_handle, raw_value_joystick_y, &voltage_value_joystick_y));
        }
        current_status.UpdateMovement(raw_value_joystick_x, raw_value_joystick_y);
        
        if (current_status.HasChanged())
        {
            ESP_LOGI(REPORT_TAG, "Current Mode: %s - Current Param: %s - Lock status: %s", 
                current_status.ModeToString(), current_status.ParamToString(), current_status.LockToString());
            ESP_LOGI(REPORT_TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, POT_ADC_CHANNEL, current_status.adc_raw);
            ESP_LOGI(REPORT_TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, POT_ADC_CHANNEL, current_status.voltage);
            ESP_LOGI(REPORT_TAG, "P: %d I: %d D: %.1f", current_status.current_P, current_status.current_I, current_status.current_D);
            ESP_LOGI(REPORT_TAG, "X: %d Y: %d", current_status.current_X, current_status.current_Y);
            
            update_display();
        } 
        else 
        {
            // ESP_LOGI(REPORT_TAG, "X: %d Y: %d", current_status.current_X, current_status.current_Y);
            update_movement_display();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Tear Down
    ESP_LOGI(MAIN_TAG, "Tearing down application...");
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_control)
    {
        example_adc_calibration_deinit(adc1_cali_control_handle);
    }
    if (do_calibration1_joy_x)
    {
        example_adc_calibration_deinit(adc1_cali_joy_x_handle);
    }
    if (do_calibration1_joy_y)
    {
        example_adc_calibration_deinit(adc1_cali_joy_y_handle);
    }
    ESP_LOGI(MAIN_TAG, "Application teardown complete.");
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