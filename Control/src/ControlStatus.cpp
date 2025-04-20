#include "ControlStatus.h"

constexpr std::array<const char*, 2> ModeStrings = { "Footing", "Fight" };
constexpr std::array<const char*, 3> ParamStrings = { "P", "I", "D" };
constexpr std::array<const char*, 2> LockStrings = { "Locked", "Unlocked" };

constexpr int JoystickMovementThreshold = 30;
constexpr int VoltageChangeThreshold = 30;

ControlStatus::ControlStatus(){};

ControlStatus::ControlStatus(float p, float i, float d, ModeType mode, ParamType param, LockType lock)
    :   current_P(p),
        current_I(i),
        current_D(d),
        current_mode(mode),
        current_param(param),
        current_lock(lock),
        adc_raw{0},
        voltage{0},
        default_X(0),
        current_X(0),
        default_Y(0),
        current_Y(0)
{
};

bool ControlStatus::MovementChanged()
{
    return this->current_X != 0 || this->current_Y != 0;
}

int ControlStatus::ControlChanged()
{
    if(abs(voltage - prev_voltage) > VoltageChangeThreshold)
    {
        if(voltage > prev_voltage)
            return 1;
        if(voltage < prev_voltage)
            return -1;
    }

    return 0;
}

bool ControlStatus::HasChanged()
{
    if (current_mode != prev_mode)
    {
        prev_mode = current_mode;
        return true;
    }
    if (current_param != prev_param)
    {
        prev_param = current_param;
        return true;
    }

    if (current_lock != prev_lock)
    {
        prev_lock = current_lock;
        return true;
    }

    if (this->ControlChanged() != 0)
    {
        prev_raw = adc_raw;
        prev_voltage = voltage;
        return true;
    }

    return this->MovementChanged();
};

// Mode
void ControlStatus::NextMode()
{
    prev_mode = static_cast<ModeType>((static_cast<u_int8_t>(this->current_mode)));
    this->current_mode = static_cast<ModeType>((static_cast<u_int8_t>(this->current_mode) + 1) % 2);
};

const char* ControlStatus::ModeToString()
{
    return ModeStrings.at(static_cast<size_t>(this->current_mode));
};

// Param
void ControlStatus::NextParam()
{
    prev_param = static_cast<ParamType>((static_cast<u_int8_t>(this->current_param)));
    this->current_param = static_cast<ParamType>((static_cast<u_int8_t>(this->current_param) + 1) % 3);
};

const char* ControlStatus::ParamToString()
{
    return ParamStrings.at(static_cast<size_t>(this->current_param));
};

// Lock
void ControlStatus::NextLock()
{
    prev_lock = static_cast<LockType>((static_cast<u_int8_t>(this->current_lock)));
    this->current_lock = static_cast<LockType>((static_cast<u_int8_t>(this->current_lock) + 1) % 2);
};

const char* ControlStatus::LockToString()
{
    return LockStrings.at(static_cast<size_t>(this->current_lock));
};

// Raw
void ControlStatus::SetRaw(int value)
{
    prev_raw = this->adc_raw;
    this->adc_raw = value;
};

// Voltage
void ControlStatus::SetVoltage(int value)
{
    prev_voltage = this->voltage;
    this->voltage = value;
};

// Movement
void ControlStatus::UpdateMovement(int x, int y)
{
    
    if (abs(x - this->default_X) > JoystickMovementThreshold)
    {
        this->current_X = x - this->default_X;
    }
    else
    {
        this->current_X = 0;
    }
    if (abs(y - this->default_Y) > JoystickMovementThreshold)
    {
        this->current_Y = y - this->default_Y;
    }
    else
    {
        this->current_Y = 0;
    }
}