#include <driver/gpio.h>
#include "driver/ledc.h"
#include "PinDefinition.h"

PinGPIODefinition::PinGPIODefinition(){};

PinGPIODefinition::PinGPIODefinition(gpio_num_t pin, gpio_mode_t mode, gpio_pulldown_t pull_down)
{
    // printf("Creating pingpio %d\n", pin);
    this->pin = pin;
    this->mode = mode;
    this->pull_down = pull_down;
};

void PinGPIODefinition::Configure()
{
    // printf("Configuring pin %d\n", pin);
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = mode;
    io_conf.pin_bit_mask = 1ULL << pin;
    io_conf.pull_down_en = pull_down;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
};

PinPWMDefinition::PinPWMDefinition(){};

PinPWMDefinition::PinPWMDefinition(gpio_num_t pin, ledc_channel_t channel, ledc_mode_t speed_mode, ledc_timer_t timer)
{
    // printf("Creating pinpwm %d\n", pin);
    this->pin = pin;
    this->channel = channel;
    this->speed_mode = speed_mode;
    this->timer = timer;
};

void PinPWMDefinition::Configure()
{
    // printf("Configuring pin %d\n", pin);
    ledc_timer_config_t ledc_timer_motor;
    ledc_timer_motor.duty_resolution = LEDC_TIMER_10_BIT;
    ledc_timer_motor.freq_hz = 1000; // Set output frequency at 1 kHz
    ledc_timer_motor.speed_mode = speed_mode;
    ledc_timer_motor.timer_num = timer;
    ledc_timer_motor.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_motor));

    ledc_channel_config_t ledc_channel_motor;
    ledc_channel_motor.channel = channel;
    ledc_channel_motor.duty = 0;
    ledc_channel_motor.gpio_num = pin;
    ledc_channel_motor.speed_mode = speed_mode;
    ledc_channel_motor.timer_sel = timer;
    ledc_channel_motor.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_motor.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_motor));
};