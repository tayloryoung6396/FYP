#include "LinearPotentiometer.hpp"

namespace module {

LinearPot linearpot1 = LinearPot(1);
LinearPot linearpot2 = LinearPot(2);

LinearPot::LinearPot(uint32_t gpio) {}

// // Get the pin state from the MCU for the pin and return it
// LinearPot::operator bool() { return bool(GPIO_PinState(gpio_pin)); }

// bool LinearPot::operator=(const bool& value) {
//     GPIO_WritePin(gpio_pin, GPIO_PinState(value));
//     return value;
// }
// bool LinearPot::operator!() {
//     GPIO_TogglePin(gpio_pin);
//     return this->operator bool();
// }
}  // namespace module