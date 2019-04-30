#ifndef MODULE_VALVE_HPP
#define MODULE_VALVE_HPP

#include <stdint.h>

namespace module {
namespace HardwareIO {
    class Valve {
    public:
        Valve(uint16_t gpio_pin);

        operator bool();
        bool operator=(const bool& value);
        bool operator!();

    private:
        uint32_t gpio_pin;
    };

    extern Valve valve1;
    extern Valve valve2;
}  // namespace HardwareIO
}  // namespace module

#endif  // MODULE_VALVE_HPP