#ifndef MODULE_LINEAR_POTENTIOMETER_HPP
#define MODULE_LINEAR_POTENTIOMETER_HPP

#include <stdint.h>
#include "utility/io/adc.hpp"

namespace module {
namespace Sensors {
    class LinearPot {
    public:
        LinearPot(uint32_t gpio, float length);

        float GetPosition();

        float ConvertPotentiometer(float position);

    private:
        uint32_t gpio;
        float length;
    };

    extern LinearPot linearpot1;
    extern LinearPot linearpot2;
}  // namespace Sensors
}  // namespace module

#endif  // MODULE_LINEAR_POTENTIOMETER_HPP