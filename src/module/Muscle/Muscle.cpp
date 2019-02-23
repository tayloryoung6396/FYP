#include "Muscle.hpp"

namespace module {

Muscle::Muscle(Valve& valve, PressureSensor& pressure_sensor) : valve(valve), pressure_sensor(pressure_sensor) {}
}  // namespace module