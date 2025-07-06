#include "RobotControl.h"
#include <array>

RobotDefinition::RobotDefinition(){};

RobotDefinition::RobotDefinition(MotorDefinition leftMotor, MotorDefinition rightMotor, PinGPIODefinition stby, int leftCorrection, int rightCorrection)
    : leftMotor(leftMotor), rightMotor(rightMotor), stby(stby), leftCorrection(leftCorrection), rightCorrection(rightCorrection) {}

constexpr std::array<const char*, 3> X_DirectionStrings = { "Right", "Center", "Left" };
constexpr std::array<const char*, 3> Y_DirectionStrings = { "Forward", "Center", "Backward" };

const char* RobotDefinition::X_DirectionToString(X_Direction dir)
{
    return X_DirectionStrings.at(static_cast<size_t>(dir));
};

const char* RobotDefinition::Y_DirectionToString(Y_Direction dir)
{
    return Y_DirectionStrings.at(static_cast<size_t>(dir));
}

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

void RobotDefinition::Stop()
{
    this->rightMotor.Stop();
    this->leftMotor.Stop();
}