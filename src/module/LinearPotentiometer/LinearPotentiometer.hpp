#ifndef MODULE_LINEAR_POTENTIOMETER_HPP
#define MODULE_LINEAR_POTENTIOMETER_HPP

#include <stdint.h>

namespace module {
class LinearPot {
public:
    LinearPot(uint32_t gpio);

    double GetPosition();

private:
    uint32_t gpio;
};

extern LinearPot linearpot1;
extern LinearPot linearpot2;
}  // namespace module

#endif  // MODULE_LINEAR_POTENTIOMETER_HPP