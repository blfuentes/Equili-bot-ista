#ifndef __PIDSERVICE_H__
#define __PIDSERVICE_H__

#include <cstdint>

class PidService {
public:
    PidService(float kp, float ki, float kd, float intSaturation = 2000);
    float update(float error, float dt);
    void reset();
private:
    float KP, KI, KD;

    float prevError;
    float accIntegral;

    float intSaturation;

    bool isFirstRun;
};

#endif // __PIDSERVICE_H__