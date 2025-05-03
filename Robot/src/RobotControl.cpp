#include "RobotControl.h"

RobotDefinition::RobotDefinition(){};

RobotDefinition::RobotDefinition(MotorDefinition rightMotor, MotorDefinition leftMotor, PinGPIODefinition stby, int leftCorrection, int rightCorrection)
    : rightMotor(rightMotor), leftMotor(leftMotor), stby(stby), leftCorrection(leftCorrection), rightCorrection(rightCorrection) {}

void RobotDefinition::Configure()
{
    // Configure the motors
    this->rightMotor.Configure();
    this->leftMotor.Configure();

    // Configure STBY pin as output
    this->stby.Configure();
    gpio_set_level(this->stby.Pin(), 1); // Set STBY high to enable the motors
}

void RobotDefinition::Drive(Direction dir, int speed)
{
    if (speed < 0)
    {
        speed = -speed; // Make speed positive for driving
    }

    if (dir.horizontal == RIGHT)
    {
        if (dir.vertical == FORWARD)
        {
            this->rightMotor.Drive(speed / 2, this->rightCorrection);
            this->leftMotor.Drive(speed, this->leftCorrection);
        }
        else if (dir.vertical == BACKWARD)
        {
            this->rightMotor.Drive(-speed / 2, this->rightCorrection);
            this->leftMotor.Drive(-speed, this->leftCorrection);
        }
        else if (dir.vertical == Y_CENTER)
        {
            this->rightMotor.Stop();
            this->leftMotor.Drive(speed, this->leftCorrection);
        }
    }
    else if (dir.horizontal == X_CENTER)
    {
        if (dir.vertical == FORWARD)
        {
            this->rightMotor.Drive(speed, this->rightCorrection);
            this->leftMotor.Drive(speed, this->leftCorrection);
        }
        else if (dir.vertical == BACKWARD)
        {
            this->rightMotor.Drive(-speed, this->rightCorrection);
            this->leftMotor.Drive(-speed, this->leftCorrection);
        }
        else if (dir.vertical == Y_CENTER)
        {
            // Stop both motors
            this->rightMotor.Stop();
            this->leftMotor.Stop();
        }
    }
    else if (dir.horizontal == LEFT)
    {
        if (dir.vertical == FORWARD)
        {
            this->rightMotor.Drive(speed, this->rightCorrection);
            this->leftMotor.Drive(speed / 2, this->leftCorrection);
        }
        else if (dir.vertical == BACKWARD)
        {
            this->rightMotor.Drive(-speed, this->rightCorrection);
            this->leftMotor.Drive(-speed / 2, this->leftCorrection);
        }
        else if (dir.vertical == Y_CENTER)
        {
            this->rightMotor.Drive(-speed, this->rightCorrection);
            this->leftMotor.Stop();
        }
    }
}