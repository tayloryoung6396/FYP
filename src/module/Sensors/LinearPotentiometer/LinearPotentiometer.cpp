#include "LinearPotentiometer.hpp"
#include <iostream>

namespace module {
namespace Sensors {

    LinearPot linearpot1 = LinearPot(1);
    LinearPot linearpot2 = LinearPot(2);

    LinearPot::LinearPot(uint32_t gpio) {}

    double LinearPot::GetPosition() {
        // Read gpio
        position = 5;
        std::cout << "Position at " << position << std::endl;
        return position;
    }
}  // namespace Sensors
}  // namespace module