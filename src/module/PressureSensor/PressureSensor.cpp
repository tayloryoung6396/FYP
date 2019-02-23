#include "PressureSensor.hpp"
namespace module {
namespace pressure_sensor {
    PressureSensor p_01 = PressureSensor();
}

PressureSensor::PressureSensor() { pressure = 0; }
}  // namespace module