#include "Valve.hpp"

namespace module {
namespace HardwareIO {

    Valve valve1 = Valve(1);
    Valve valve2 = Valve(2);

    Valve::Valve(uint32_t gpio) {}

    // // Get the pin state from the MCU for the pin and return it
    // Valve::operator bool() { return bool(GPIO_PinState(gpio_pin)); }

    // bool Valve::operator=(const bool& value) {
    //     GPIO_WritePin(gpio_pin, GPIO_PinState(value));
    //     return value;
    // }
    // bool Valve::operator!() {
    //     GPIO_TogglePin(gpio_pin);
    //     return this->operator bool();
    // }
}  // namespace HardwareIO
}  // namespace module