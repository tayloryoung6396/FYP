#include "LinearPotentiometer.hpp"
#include <cmath>
#include <iostream>
#include "utility/io/uart.hpp"

namespace module {
namespace Sensors {

    LinearPot linearpot1 = LinearPot(1, 0.1);
    LinearPot linearpot2 = LinearPot(3, 0.1);

    LinearPot::LinearPot(uint32_t gpio, float length) : gpio(gpio), length(length) {}

    float LinearPot::GetPosition() {
        // Read gpio
        float position = ConvertPotentiometer(utility::io::adc_io.GetSensors(gpio));
        return position;
    }

    float LinearPot::ConvertPotentiometer(float position) {
        const float scale = std::pow(2, 12);
        position          = position / scale * length;
        return position;
    }
}  // namespace Sensors
}  // namespace module