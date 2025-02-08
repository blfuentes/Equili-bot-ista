#include "servo.h"
#include <esp_err.h>
#include <driver/ledc.h>

Servo::Servo()
{

};

void Servo::initHw()
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = GPIO_NUM_15,
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
        .sleep_mode     = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD
    };

    uint32_t neutralDuty = (minDuty + maxDuty) / 2;

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, neutralDuty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
};

void Servo::calibrate(uint32_t min, uint32_t max)
{
    /**
     * duty = pulse width(micros) * frequency(Hz) * resolution / 1_000_000
     *  T 0.5ms -> 0
     *  T 1ms -> 45
     *  T 1.5ms -> 90
     *  T 2ms -> 135
     *  T 2.5ms -> 180
     * minduty = 500 * 50 * 16384 / 1_000_000 = 409
     * maxduty = 2500 * 50 * 16384 / 1_000_000 = 2048
     */
    // given
    minDuty = min;
    maxDuty = max;
};

void Servo::setPos(uint32_t pos)
{
    uint32_t duty = minDuty + (pos * (maxDuty - minDuty)) / 180;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
};