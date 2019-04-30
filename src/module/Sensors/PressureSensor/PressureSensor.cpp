#include "PressureSensor.hpp"
#include <iostream>

namespace module {
namespace Sensors {
    PressureSensor pressuresensor1 = PressureSensor(1);
    // PressureSensor pressuresensor2  = PressureSensor(2);
    // PressureSensor pressuresensor3  = PressureSensor(3);
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

    double PressureSensor::GetPressure() {
        pressure = 10;
        std::cout << "Pressure at " << pressure << std::endl;
        pressure = ConvertPressure(shared::utility::adc_io.GetSensors(gpio));
        return pressure;
    }

    double PressureSensor::ConvertPressure(double Pressure) {

        Pressure = Pressure;  // TODO Scale by the adc ammount and then the data sheet specification
        return Pressure;
    }
}  // namespace Sensors
}  // namespace module