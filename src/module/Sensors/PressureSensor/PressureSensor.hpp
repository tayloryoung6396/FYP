#ifndef MODULE_PRESSURE_SENSOR_HPP
#define MODULE_PRESSURE_SENSOR_HPP

#include <stdint.h>
#include "utility/io/adc.hpp"

namespace module {
namespace Sensors {
    class PressureSensor {
    public:
        PressureSensor(uint32_t gpio);

        double GetPressure();

        double ConvertPressure(double Pressure);

    private:
        uint32_t gpio;
        uint32_t pressure;
    };

    extern PressureSensor pressuresensor1;
    extern PressureSensor pressuresensor2;
    extern PressureSensor pressuresensor3;
    // extern PressureSensor pressuresensor4;
    // extern PressureSensor pressuresensor5;
    // extern PressureSensor pressuresensor6;
    // extern PressureSensor pressuresensor7;
    // extern PressureSensor pressuresensor8;
    // extern PressureSensor pressuresensor9;
    // extern PressureSensor pressuresensor10;
    // extern PressureSensor pressuresensor11;
    // extern PressureSensor pressuresensor12;
}  // namespace Sensors
}  // namespace module

#endif  // MODULE_PRESSURE_SENSOR_HPP