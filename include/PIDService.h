#include <cstdint>
#ifndef __PIDSERVICE_H__
#define __PIDSERVICE_H__

class PidService {
public:
    PidService(float pKp, float pKi, float pKd, float pInt_saturation = 2000);

    float update(float error, float dt);

    void reset();

    void setKp(float pKp) { KP = pKp; }
    void setKi(float pKi) { KI = pKi; }
    void setKd(float pKd) { KD = pKd; }

private:
    float KP, KI, KD;

    float prev_error;
    float acum_integral;

    float int_saturation;

    uint8_t first_run;
};

#endif // __PIDSERVICE_H__