#ifndef MODULE_MUSCLE_HPP
#define MODULE_MUSCLE_HPP

#include "../../../shared/utility/PID/PID.hpp"
#include "../../Sensors/LinearPotentiometer/LinearPotentiometer.hpp"
#include "../../Sensors/PressureSensor/PressureSensor.hpp"
#include "../Valve/Valve.hpp"

namespace module {
namespace HardwareIO {
    class Muscle {
    public:
        Muscle(Valve& valve,
               Sensors::PressureSensor& pressure_sensor,
               Sensors::LinearPot& linear_pot,
               shared::utility::PID& pid);

        void SetPosition(double set_point);

        double GetPosition();

        double GetPressure();

    private:
        Valve& valve;
        Sensors::PressureSensor& pressure_sensor;
        Sensors::LinearPot& linear_pot;
        shared::utility::PID& pid;

        double contraction;
    };
}  // namespace HardwareIO
}  // namespace module

#endif  // MODULE_MUSCLE_HPP