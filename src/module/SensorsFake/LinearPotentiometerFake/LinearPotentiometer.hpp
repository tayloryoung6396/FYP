#ifndef MODULE_LINEAR_POTENTIOMETER_FAKE_HPP
#define MODULE_LINEAR_POTENTIOMETER_FAKE_HPP

#include <stdint.h>

namespace module {
namespace Sensors {
    class LinearPot {
    public:
        LinearPot(float length, float position);

        float GetPosition();

        float ConvertPotentiometer(float position);

    private:
        float position;
        float length;
    };

    extern LinearPot linearpot1;
    extern LinearPot linearpot2;
}  // namespace Sensors
}  // namespace module

#endif  // MODULE_LINEAR_POTENTIOMETER_FAKE_HPP