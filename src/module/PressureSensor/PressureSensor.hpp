#ifndef MODULE_PRESSURE_SENSOR_HPP
#define MODULE_PRESSURE_SENSOR_HPP

#include <stdint.h>

namespace module {
class PressureSensor {
public:
    PressureSensor(uint32_t gpio);

    double Read();

private:
    uint32_t gpio;
    uint32_t pressure;
};

namespace Sensors {
    extern PressureSensor pressuresensor1;
    extern PressureSensor pressuresensor2;
}  // namespace Sensors
}  // namespace module

#endif  // MODULE_PRESSURE_SENSOR_HPP