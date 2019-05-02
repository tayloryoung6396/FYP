#ifndef MODULE_LINEAR_POTENTIOMETER_HPP
#define MODULE_LINEAR_POTENTIOMETER_HPP

#include <stdint.h>
#include "utility/io/adc.hpp"

namespace module {
namespace Sensors {
    class LinearPot {
    public:
        LinearPot(uint32_t gpio, double length);

        double GetPosition();

        double ConvertPotentiometer(double position);

    private:
        uint32_t gpio;
        double length;
    };

    extern LinearPot linearpot1;
    // extern LinearPot linearpot2;
    // extern LinearPot linearpot3;
    // extern LinearPot linearpot4;
    // extern LinearPot linearpot5;
    // extern LinearPot linearpot6;
    // extern LinearPot linearpot7;
    // extern LinearPot linearpot8;
    // extern LinearPot linearpot9;
    // extern LinearPot linearpot10;
    // extern LinearPot linearpot11;
    // extern LinearPot linearpot12;
}  // namespace Sensors
}  // namespace module

#endif  // MODULE_LINEAR_POTENTIOMETER_HPP