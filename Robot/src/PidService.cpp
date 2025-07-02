#include "PIDService.h"

PidService::PidService(float pKp, float pKi, float pKd, float pInt_saturation ):
    kp(pKp), ki(pKi), kd(pKd), intSaturation(pInt_saturation)
{
    accIntegral = 0;
    firstRun = 1;
    prevD = 0;
}

float PidService::update(float error, float dt)
{

    if(firstRun)
    {
        prevError = error;
        firstRun = 0;
    }

    float prop = error;

    accIntegral += (error + prevError) * 0.5f * dt;

    if(error * prevError < 0)
    {
        accIntegral = 0;
    }

    if( accIntegral > intSaturation)
    {
        accIntegral = intSaturation;
    }
    else if (accIntegral < -intSaturation)
    {
        accIntegral = -intSaturation;
    }

    float derivative = prevD * 0.2 + ((error - prevError) / dt) * 0.7;
    prevD = derivative;

    prevError = error;

    return kp * error + ki * accIntegral + kd * derivative;
}

void PidService::reset(void)
{
    firstRun = 1;
    ki = 0;
    prevError = 0;
};