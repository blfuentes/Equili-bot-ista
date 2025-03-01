#ifndef __SERVO_H__
#define __SERVO_H__

#include <driver/gpio.h>
#include <stdint.h>

class Servo {
public:
    Servo();

    void initHw(gpio_num_t servo_pin);

    void calibrate(uint32_t min, uint32_t max);

    void setPos(uint32_t pos);
private:
    // arduino servo 180º 3.3v
    uint32_t minDuty = 400;  // 1 ms pulse width (0 degrees)
    uint32_t maxDuty = 1900; // 2 ms pulse width (180 degrees)
    // sg90 180º 5v
    // uint32_t minDuty = 409;  // 1 ms pulse width (0 degrees)
    // uint32_t maxDuty = 2048; // 2 ms pulse width (180 degrees)
};

#endif //__SERVO_H__
