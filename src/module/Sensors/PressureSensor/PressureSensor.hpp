#ifndef MODULE_PRESSURE_SENSOR_HPP
#define MODULE_PRESSURE_SENSOR_HPP

#include <stdint.h>
#include "utility/io/adc.hpp"

namespace module {
namespace Sensors {
    class PressureSensor {
    public:
        PressureSensor(uint32_t gpio);

        float GetPressure();

        float ConvertPressure(float Pressure);

    private:
        uint32_t gpio;
        uint32_t pressure;
    };

    extern PressureSensor pressuresensor1;
    extern PressureSensor pressuresensor2;
    extern PressureSensor pressuresensor3;
}  // namespace Sensors
}  // namespace module

#endif  // MODULE_PRESSURE_SENSOR_HPP