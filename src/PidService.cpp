#include "PIDService.h"

PidService::PidService(float pKp, float pKi, float pKd, float pInt_saturation ):
    KP(pKp), KI(pKi), KD(pKd), int_saturation(pInt_saturation)
{
    acum_integral = 0;
    first_run = 1;
}

float PidService::update(float error, float dt)
{

    if(first_run)
    {
        prev_error = error;
        first_run = 0;
    }

    float prop = error;
    
    acum_integral += (error + prev_error)*0.5f*dt;

    if( acum_integral > int_saturation)
    {
        acum_integral = int_saturation;
    }
    else if (acum_integral < -int_saturation)
    {
        acum_integral = -int_saturation;
    }

    float derivative = (error - prev_error)/dt;

    prev_error = error;

    return KP*error + KI*acum_integral + KD*derivative;
}

void PidService::reset(void)
{
    first_run = 1;
    KI = 0;
    prev_error = 0;
};