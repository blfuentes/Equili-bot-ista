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

enum class LockType : u_int8_t {
    LOCKED = 0,
    UNLOCKED = 1
};

class ControlStatus {
    private:
        ModeType prev_mode;
        ParamType prev_param;
        LockType prev_lock;
        int prev_raw;
        int prev_voltage;
    public:
        int current_P;
        int current_I;
        float current_D;
        ModeType current_mode;
        ParamType current_param;
        LockType current_lock;
        int adc_raw;
        int voltage;

        int default_X;
        int current_X;
        int default_Y;
        int current_Y;

        ControlStatus();
        ControlStatus(int p, int i, float d, ModeType mode, ParamType param, LockType lock);

        int ControlChanged();
        bool HasChanged();
        void NextMode();
        const char* ModeToString();
        void NextParam();
        const char* ParamToString();
        void NextLock();
        const char* LockToString();
        void SetRaw(int value);
        void SetVoltage(int value);
        bool MovementChanged();
        void UpdateMovement(int x, int y);
};


#endif // __CONTROLSTATUS_H__