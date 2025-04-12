#include <sys/types.h>
#include <array>
#include <stdexcept>
#ifndef __CONTROLSTATUS_H__
#define __CONTROLSTATUS_H__

enum class ModeType : u_int8_t {
    STANDING = 0,
    WAR_MODE = 1
};

enum class ParamType : u_int8_t {
    P = 0,
    I = 1,
    D = 2
};

class ControlStatus {
    private:
        ModeType prev_mode;
        ParamType prev_param;
        int prev_raw;
        int prev_voltage;
    public:
        ModeType current_mode;
        ParamType current_param;
        int adc_raw;
        int voltage;
        // std::array<std::array<int, 10>, 2> adc_raw{};
        // std::array<std::array<int, 10>, 2> voltage{};

        ControlStatus();
        ControlStatus(ModeType mode, ParamType param);

        bool HasChanged();
        void NextMode();
        const char* ModeToString();
        void NextParam();
        const char* ParamToString();
        void SetRaw(int value);
        void SetVoltage(int value);
};


#endif // __CONTROLSTATUS_H__