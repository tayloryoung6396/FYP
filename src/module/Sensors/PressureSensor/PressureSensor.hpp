#ifndef MODULE_PRESSURE_SENSOR_HPP
#define MODULE_PRESSURE_SENSOR_HPP

#include <stdint.h>

namespace module {
namespace Sensors {
    class PressureSensor {
    public:
        PressureSensor(uint32_t gpio);

        double GetPressure();

    private:
        uint32_t gpio;
        uint32_t pressure;
    };


    extern PressureSensor pressuresensor1;
    extern PressureSensor pressuresensor2;
}  // namespace Sensors
}  // namespace module

#endif  // MODULE_PRESSURE_SENSOR_HPP