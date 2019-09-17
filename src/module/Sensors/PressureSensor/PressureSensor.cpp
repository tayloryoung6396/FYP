#include "PressureSensor.hpp"
#include <iostream>
#include "utility/io/uart.hpp"

namespace module {
namespace Sensors {
    PressureSensor pressuresensor1 = PressureSensor(4);
    PressureSensor pressuresensor2 = PressureSensor(6);
    PressureSensor pressuresensor3 = PressureSensor(8);
    // PressureSensor pressuresensor4  = PressureSensor(4);
    // PressureSensor pressuresensor5  = PressureSensor(5);
    // PressureSensor pressuresensor6  = PressureSensor(6);
    // PressureSensor pressuresensor7  = PressureSensor(7);
    // PressureSensor pressuresensor8  = PressureSensor(8);
    // PressureSensor pressuresensor9  = PressureSensor(9);
    // PressureSensor pressuresensor10 = PressureSensor(10);
    // PressureSensor pressuresensor11 = PressureSensor(11);
    // PressureSensor pressuresensor12 = PressureSensor(12);

    PressureSensor::PressureSensor(uint32_t gpio) : gpio(gpio) {}

    float PressureSensor::GetPressure() {
        pressure = ConvertPressure(utility::io::adc_io.GetSensors(gpio));
        utility::io::debug.out("Pressure gpio %d at %lf\n", gpio, pressure);
        return pressure;
    }

    float PressureSensor::ConvertPressure(float Pressure) {
        return Pressure;  // TODO Scale by the adc ammount and then the data sheet specification
    }
}  // namespace Sensors
}  // namespace module