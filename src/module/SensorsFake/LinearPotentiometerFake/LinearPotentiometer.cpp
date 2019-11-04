#include <math.h>
#include <iostream>
#include "LinearPotentiometerFake.hpp"
#include "utility/io/uart.hpp"

namespace module {
namespace Sensors {

    LinearPot linearpot1 = LinearPot(0.1, 50);
    LinearPot linearpot2 = LinearPot(0.1, 50);

    LinearPot::LinearPot(float length) : length(length), position(position) {}

    float LinearPot::GetPosition() {
        // Read gpio
        position;  //= ConvertPotentiometer(utility::io::adc_io.GetSensors(gpio));
        return position;
    }

    float LinearPot::ConvertPotentiometer(float position) {
        const float scale = 4096;
        position          = position / scale * length;
        return position;
    }
}  // namespace Sensors
}  // namespace module