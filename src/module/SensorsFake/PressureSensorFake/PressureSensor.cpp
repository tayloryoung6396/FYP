#include <iostream>
#include "PressureSensorFake.hpp"
#include "utility/io/uart.hpp"

namespace module {
namespace Sensors {
    PressureSensor pressuresensor1 = PressureSensor(413685);
    PressureSensor pressuresensor2 = PressureSensor(413685);
    PressureSensor pressuresensor3 = PressureSensor(413685);

    PressureSensor::PressureSensor(uint32_t pressure) : pressure(pressure) {}

    float PressureSensor::GetPressure() {
        pressure;  //= ConvertPressure(utility::io::adc_io.GetSensors(gpio));
        return pressure;
    }

    float PressureSensor::ConvertPressure(float Pressure) {
        return Pressure;  // TODO Scale by the adc ammount and then the data sheet specification
    }
}  // namespace Sensors
}  // namespace module