#ifndef MODULE_LINEAR_POTENTIOMETER_HPP
#define MODULE_LINEAR_POTENTIOMETER_HPP

#include <stdint.h>

namespace module {
namespace Sensors {
    class LinearPot {
    public:
        LinearPot(uint32_t gpio);

        double GetPosition();

    private:
        uint32_t gpio;
        double position;
    };

    extern LinearPot linearpot1;
    extern LinearPot linearpot2;
}  // namespace Sensors
}  // namespace module

#endif  // MODULE_LINEAR_POTENTIOMETER_HPP