#include "Muscle.hpp"
#include <iostream>

namespace module {

Muscle::Muscle(Valve& valve, PressureSensor& pressure_sensor, LinearPot& linear_pot, PID& pid)
    : valve(valve), pressure_sensor(pressure_sensor), linear_pot(linear_pot), pid(pid) {}

void Muscle::SetPosition(double set_point) {
    std::cout << "Set Point " << set_point << std::endl;

    double pressure = pressure_sensor.GetPressure();
    double position = linear_pot.GetPosition();

    pid.Compute();
}

double Muscle::GetPosition() { return linear_pot.GetPosition(); }

double Muscle::GetPressure() { return pressure_sensor.GetPressure(); }
}  // namespace module