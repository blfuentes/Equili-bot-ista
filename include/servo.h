#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>

class Servo {
public:
    Servo();

    void initHw();

    void calibrate(uint32_t min, uint32_t max);

    void setPos(uint32_t pos);
private:
    uint32_t minDuty = 400;  // 1 ms pulse width (0 degrees)
    uint32_t maxDuty = 1900; // 2 ms pulse width (180 degrees)
    // uint32_t minDuty = 409;  // 1 ms pulse width (0 degrees)
    // uint32_t maxDuty = 2048; // 2 ms pulse width (180 degrees)
};

#endif //__SERVO_H__
