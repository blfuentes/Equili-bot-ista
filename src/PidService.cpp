#include "PIDService.h"

PidService::PidService(float kp, float ki, float kd, float intSaturation) :
    KP(kp), KI(ki), KD(kd), intSaturation(intSaturation)
{
    accIntegral = 0;
    isFirstRun = true;
}

float PidService::update(float error, float dt) {
    if (dt <= 0) {
        // Handle invalid dt (e.g., return 0 or previous output)
        return 0;
    }

    if (isFirstRun) {
        prevError = error;
        isFirstRun = false;
    }

    // Proportional term
    float prop = error;

    // Integral term (with trapezoidal integration and saturation)
    accIntegral += (error + prevError) * 0.5f * dt;
    if (accIntegral > intSaturation) {
        accIntegral = intSaturation;
    } else if (accIntegral < -intSaturation) {
        accIntegral = -intSaturation;
    }

    // Derivative term (with low-pass filter if needed)
    float derivative = (error - prevError) / dt;

    // Update previous error
    prevError = error;

    // Calculate PID output
    return KP * prop + KI * accIntegral + KD * derivative;
}

void PidService::reset(void)
{
    isFirstRun = true;
    KI = 0;
    prevError = 0;
};