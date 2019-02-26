#include "Muscle.hpp"
#include <iostream>

namespace module {
namespace HardwareIO {

    Muscle::Muscle(Valve& valve,
                   Sensors::PressureSensor& pressure_sensor,
                   Sensors::LinearPot& linear_pot,
                   shared::utility::PID& pid)
        : valve(valve), pressure_sensor(pressure_sensor), linear_pot(linear_pot), pid(pid) {}

    void Muscle::SetPosition(double set_point) {
        std::cout << "Set Point " << set_point << std::endl;

        double pressure = pressure_sensor.GetPressure();
        double position = linear_pot.GetPosition();

        pid.Compute(set_point);
    }

    double Muscle::GetPosition() { return linear_pot.GetPosition(); }

    double Muscle::GetPressure() { return pressure_sensor.GetPressure(); }

    bool Muscle::GetValveState() {  // return (valve.bool());
    }
}  // namespace HardwareIO
}  // namespace module
