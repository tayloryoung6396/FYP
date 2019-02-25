#ifndef MODULE_MUSCLE_HPP
#define MODULE_MUSCLE_HPP

#include "../LinearPotentiometer/LinearPotentiometer.hpp"
#include "../PID/PID.hpp"
#include "../PressureSensor/PressureSensor.hpp"
#include "../Valve/Valve.hpp"

namespace module {
class Muscle {
public:
    Muscle(Valve& valve, PressureSensor& pressure_sensor, LinearPot& linear_pot, PID& pid);

    void SetPosition(double set_point);

private:
    Valve& valve;
    PressureSensor& pressure_sensor;
    LinearPot& linear_pot;
    PID& pid;
};
}  // namespace module

#endif  // MODULE_MUSCLE_HPP