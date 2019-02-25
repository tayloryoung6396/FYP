#ifndef MODULE_LINEAR_POTENTIOMETER_HPP
#define MODULE_LINEAR_POTENTIOMETER_HPP

#include <stdint.h>

namespace module {
class LinearPot {
public:
    LinearPot(uint32_t gpio);

    // operator bool();
    // bool operator=(const bool& value);
    // bool operator!();

private:
    uint32_t gpio;

    double contraction;
};

extern LinearPot linearpot1;
extern LinearPot linearpot2;
}  // namespace module

#endif  // MODULE_LINEAR_POTENTIOMETER_HPP