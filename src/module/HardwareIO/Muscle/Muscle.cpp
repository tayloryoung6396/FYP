#include "Muscle.hpp"
#include <iostream>

namespace module {
namespace HardwareIO {

    Muscle::Muscle(module::HardwareIO::muscle_t muscle)
        : valve(muscle.valve)
        , pressure_sensor(muscle.pressure_sensor)
        , linear_pot(muscle.linear_pot)
        , pid(muscle.pid)
        , length(muscle.length) {}

    void Muscle::SetPosition(double set_point) {
        std::cout << "Set Point " << set_point << std::endl;

        double pressure = pressure_sensor.GetPressure();
        double position = linear_pot.GetPosition();

        double value   = position;
        double control = pid.Compute(set_point, value);

        std::cout << "Control " << control << std::endl;

        // From the control point decide how to act on the valve to reach a required state
    }

    double Muscle::GetPosition() { return linear_pot.GetPosition(); }

    double Muscle::GetPressure() { return pressure_sensor.GetPressure(); }

    bool Muscle::GetValveState() { return valve; }
}  // namespace HardwareIO
}  // namespace module