#ifndef __ROBOTCONTROL_H__
#define __ROBOTCONTROL_H__

#include "MotorControl.h"

typedef enum {
    RIGHT = 0,
    X_CENTER,
    LEFT,
} X_Direction;

typedef enum {
    FORWARD = 0,
    Y_CENTER,
    BACKWARD,
} Y_Direction;

struct Direction {
    X_Direction horizontal;
    Y_Direction vertical;
};

class RobotDefinition {
    MotorDefinition rightMotor;
    MotorDefinition leftMotor;

    PinGPIODefinition stby;

    int leftCorrection;
    int rightCorrection;

    public:
        RobotDefinition();
        RobotDefinition(MotorDefinition rightMotor, MotorDefinition leftMotor, PinGPIODefinition stby, int leftCorrection, int rightCorrection);

        void Configure();
        void Drive(Direction dir, int speed);
        void Stop();
};


#endif // __ROBOTCONTROL_H__