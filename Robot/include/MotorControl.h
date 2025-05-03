#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_err.h>

#include "PinDefinition.h"

constexpr int DEFAULT_SPEED = 392;

// Motor definition
class MotorDefinition {
    PinGPIODefinition in1Def;
    PinGPIODefinition in2Def;
    PinPWMDefinition pwmDef;
    ledc_channel_t channel;
    ledc_mode_t speedMode;
    ledc_timer_t timer;
    uint8_t in1Level;
    uint8_t in2Level;

public:
    MotorDefinition();
    MotorDefinition(gpio_num_t in1, gpio_num_t in2, uint8_t in1_level, uint8_t in2_level, gpio_num_t pwm, ledc_channel_t channel, ledc_mode_t speed_mode, ledc_timer_t timer);
    void Configure();
    void Drive(int speed, int correction = 0);
    void Stop();
private:
    typedef enum{
        DIR_FWD = 0,
        DIR_BCK
    }Direction;
    Direction dir; 
};

#endif // __MOTORCONTROL_H__