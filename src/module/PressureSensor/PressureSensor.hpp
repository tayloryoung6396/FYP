#ifndef MODULE_PRESSURE_SENSOR_HPP
#define MODULE_PRESSURE_SENSOR_HPP

#include <stdint.h>

namespace module {
class PressureSensor {
public:
    PressureSensor();

private:
    uint32_t pressure;
};

namespace pressure_sensor {
    extern PressureSensor p_01;
}  // namespace pressure_sensor
}  // namespace module

#endif  // MODULE_PRESSURE_SENSOR_HPP