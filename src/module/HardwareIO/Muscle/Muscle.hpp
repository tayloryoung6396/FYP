#ifndef MODULE_MUSCLE_HPP
#define MODULE_MUSCLE_HPP

#include "HardwareIO/Valve/Valve.hpp"
#include "Sensors/LinearPotentiometer/LinearPotentiometer.hpp"
#include "Sensors/PressureSensor/PressureSensor.hpp"
#include "utility/PID/PID.hpp"

namespace module {
namespace HardwareIO {

    struct muscle_t {
        Valve& valve;
        Sensors::PressureSensor& pressure_sensor;
        Sensors::LinearPot& linear_pot;
        shared::utility::PID& pid;
        double length;
    };

    class Muscle {
    public:
        Muscle(module::HardwareIO::muscle_t muscle);

        void SetPosition(double set_point);

        double GetPosition();

        double GetPressure();

        bool GetValveState();

    private:
        double contraction;

        Valve& valve;
        Sensors::PressureSensor& pressure_sensor;
        Sensors::LinearPot& linear_pot;
        shared::utility::PID& pid;
        double length;
    };
}  // namespace HardwareIO
}  // namespace module

#endif  // MODULE_MUSCLE_HPP