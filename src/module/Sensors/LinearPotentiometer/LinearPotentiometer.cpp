#include "LinearPotentiometer.hpp"
#include <cmath>
#include <iostream>
#include "utility/io/uart.hpp"

namespace module {
namespace Sensors {

    LinearPot linearpot1 = LinearPot(1, 0.1);
    // LinearPot linearpot2  = LinearPot(2);
    // LinearPot linearpot3  = LinearPot(3);
    // LinearPot linearpot4  = LinearPot(4);
    // LinearPot linearpot5  = LinearPot(5);
    // LinearPot linearpot6  = LinearPot(6);
    // LinearPot linearpot7  = LinearPot(7);
    // LinearPot linearpot8  = LinearPot(8);
    // LinearPot linearpot9  = LinearPot(9);
    // LinearPot linearpot10 = LinearPot(10);
    // LinearPot linearpot11 = LinearPot(11);
    // LinearPot linearpot12 = LinearPot(12);

    LinearPot::LinearPot(uint32_t gpio, double length) : length(length) {}

    double LinearPot::GetPosition() {
        // Read gpio
        double position = ConvertPotentiometer(utility::io::adc_io.GetSensors(gpio));
        utility::io::debug.out("Position at %lf\n", position);
        return position;
    }

    double LinearPot::ConvertPotentiometer(double position) {
        double scale = std::pow(2, 12);
        position     = position / scale * length;
        return position;
    }
}  // namespace Sensors
}  // namespace module