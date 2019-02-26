#include "LinearPotentiometer.hpp"

namespace module {
namespace Sensors {

    LinearPot linearpot1 = LinearPot(1);
    LinearPot linearpot2 = LinearPot(2);

    LinearPot::LinearPot(uint32_t gpio) {}

    double LinearPot::GetPosition() {
        // Read gpio
    }
}  // namespace Sensors
}  // namespace module