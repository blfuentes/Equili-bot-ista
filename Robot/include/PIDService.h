#ifndef __PIDSERVICE_H__
#define __PIDSERVICE_H__

#include <cstdint>

class PidService {
public:
    PidService(float pKp, float pKi, float pKd, float pInt_saturation = 2000);

    float update(float error, float dt);

    void reset();

    void setKp(float pKp){ kp = pKp; }
    void setKi(float pKi){ ki = pKi; }
    void setKd(float pKd){ kd = pKd; }

    void getLastPid(float &p, float &i, float &d){
        p = prevError*kp;
        i = accIntegral*ki;
        d = prevD*kd;
    }

private:
    float kp, ki, kd;

    float prevError;
    float prevD;
    float accIntegral;

    float intSaturation;

    uint8_t firstRun;
};

#endif // __PIDSERVICE_H__