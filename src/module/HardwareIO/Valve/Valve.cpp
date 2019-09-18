#include "Valve.hpp"
#include "utility/io/gpio.hpp"

namespace module {
namespace HardwareIO {

    Valve valve1 = Valve(1);
    Valve valve2 = Valve(2);

    Valve::Valve(uint16_t gpio_pin) : gpio_pin(gpio_pin) {}

    // // Get the pin state from the MCU for the pin and return it
    Valve::operator bool() { return gpio_pin; }

    bool Valve::operator=(const bool& value) {
        gpio_pin = value;
        return value;
    }
    bool Valve::operator!() { return this->operator bool(); }
}  // namespace HardwareIO
}  // namespace module
