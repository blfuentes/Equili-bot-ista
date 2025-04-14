#include "ControlStatus.h"

constexpr std::array<const char*, 2> ModeStrings = { "Standing", "Fight" };
constexpr std::array<const char*, 3> ParamStrings = { "P", "I", "D" };

ControlStatus::ControlStatus(){};

ControlStatus::ControlStatus(int p, int i, float d, ModeType mode, ParamType param)
    :   current_P(p),
        current_I(i),
        current_D(d),
        current_mode(mode),
        current_param(param),
        adc_raw{0},
        voltage{0}
{
};

int ControlStatus::ControlChanged()
{
    if(abs(voltage - prev_voltage) > 15)
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

    if (this->ControlChanged() != 0)
    {
        prev_raw = adc_raw;
        prev_voltage = voltage;
        return true;
    }

    return false;
};

// Mode
void ControlStatus::NextMode()
{
    prev_mode = static_cast<ModeType>((static_cast<u_int8_t>(this->current_mode)));
    this->current_mode = static_cast<ModeType>((static_cast<u_int8_t>(this->current_mode) + 1) % 2);
    // printf("Mode changed from %u to %u\n", static_cast<u_int8_t>(prev_mode), static_cast<u_int8_t>(this->current_mode));
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
    // printf("Param changed from %u to %u\n", static_cast<u_int8_t>(prev_param), static_cast<u_int8_t>(this->current_param));
};

const char* ControlStatus::ParamToString()
{
    return ParamStrings.at(static_cast<size_t>(this->current_param));
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