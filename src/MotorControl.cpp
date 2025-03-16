#include "MotorControl.h"
#include "PinDefinition.h"

MotorDefinition::MotorDefinition(){};

MotorDefinition::MotorDefinition(gpio_num_t in1, gpio_num_t in2, uint8_t in1_level, uint8_t in2_level, gpio_num_t pwm, ledc_channel_t channel, ledc_mode_t speed_mode, ledc_timer_t timer)
{
    // printf("Creating motor\n");
    this->in1Def = PinGPIODefinition(in1, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE);
    this->in2Def = PinGPIODefinition(in2, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE);
    this->pwmDef = PinPWMDefinition(pwm, channel, speed_mode, timer);
    this->channel = channel;
    this->speedMode = speed_mode;
    this->timer = timer;
    this->in1Level = in1_level;
    this->in2Level = in2_level;
    this->dir = DIR_FWD;
};

void MotorDefinition::Configure()
{
    // printf("Configuring motor\n");
    // Configure MOTOR_IN1 pin as output
    this->in1Def.Configure();

    // Configure MOTOR_IN_2 pin as output
    this->in2Def.Configure();

    // Prepare and then apply the LEDC PWM timer configuration
    this->pwmDef.Configure();
};

void MotorDefinition::Drive(int speed)
{
    // Determine the direction based on the sign of speed
    if (speed < 0) {
        // Forward direction
        gpio_set_level(this->in1Def.Pin(), this->in1Level);
        gpio_set_level(this->in2Def.Pin(), this->in2Level); // Opposite of in1Level
        dir = DIR_FWD;
    } else if (speed > 0) {
        // Backward direction
        gpio_set_level(this->in1Def.Pin(), !this->in1Level); // Opposite of in1Level
        gpio_set_level(this->in2Def.Pin(), !this->in2Level);
        dir = DIR_BCK;
    } else {
        // Stop the motor
        Stop();
    }

    // Set the PWM duty cycle (absolute value of speed)
    int inverted_duty = 1023 - abs(speed);
    ledc_set_duty(this->speedMode, this->channel, abs(inverted_duty));
    ledc_update_duty(this->speedMode, this->channel);
}

void MotorDefinition::Stop()
{
    // printf("Stopping motor\n");
    gpio_set_level(this->in1Def.Pin(), 0);
    gpio_set_level(this->in2Def.Pin(), 0);
    ledc_set_duty(this->speedMode, this->channel, 0);
    ledc_update_duty(this->speedMode, this->channel);
};