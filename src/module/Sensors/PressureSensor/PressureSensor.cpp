#include "PressureSensor.hpp"
#include <iostream>
#include "utility/io/uart.hpp"

namespace module {
namespace Sensors {
    PressureSensor pressuresensor1 = PressureSensor(4);
    PressureSensor pressuresensor2 = PressureSensor(6);
    PressureSensor pressuresensor3 = PressureSensor(8);

    PressureSensor::PressureSensor(uint32_t gpio) : gpio(gpio) {}

    float PressureSensor::GetPressure() {
        pressure = ConvertPressure(utility::io::adc_io.GetSensors(gpio));
        return pressure;
    }

    float PressureSensor::ConvertPressure(float Pressure) {
        return Pressure;  // TODO Scale by the adc ammount and then the data sheet specification
    }
}  // namespace Sensors
}  // namespace module