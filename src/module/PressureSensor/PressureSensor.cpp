#include "PressureSensor.hpp"
#include <iostream>

namespace module {

namespace Sensors {
    PressureSensor pressuresensor1 = PressureSensor(1);
    PressureSensor pressuresensor2 = PressureSensor(2);
}  // namespace Sensors
PressureSensor::PressureSensor(uint32_t gpio) {}

double PressureSensor::Read() {
    pressure = 10;
    std::cout << "Pressure at " << pressure << std::endl;
    return pressure;
}
}  // namespace module