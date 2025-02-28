#ifndef __PINDEFINITION_H__
#define __PINDEFINITION_H__

#include <driver/gpio.h>
#include "driver/ledc.h"

class PinBaseDefinition {
protected:
    gpio_num_t pin;
public:
    virtual void Configure() = 0;
    gpio_num_t Pin() { return pin; }
};

class PinGPIODefinition : public PinBaseDefinition {
    gpio_mode_t mode;
    gpio_pulldown_t pull_down;

public:
    PinGPIODefinition();
    PinGPIODefinition(gpio_num_t pin, gpio_mode_t mode, gpio_pulldown_t pull_down);

    void Configure();
};

class PinPWMDefinition : public PinBaseDefinition {
    ledc_channel_t channel;
    ledc_mode_t speed_mode;
    ledc_timer_t timer;

public:
    PinPWMDefinition();
    PinPWMDefinition(gpio_num_t pin, ledc_channel_t channel, ledc_mode_t speed_mode, ledc_timer_t timer);
    
    void Configure();
};

#endif // __PINDEFINITION_H__