#include "PID.hpp"

namespace module {

PID pid1 = PID();
PID pid2 = PID();

PID::PID() {}  // PressureSensor& pressure_sensor, LinearPot& linear_pot)
//: pressure_sensor(pressure_sensor), linear_pot(linear_pot) {}

void PID::Compute() {}

// // Get the pin state from the MCU for the pin and return it
// PID::operator bool() { return bool(GPIO_PinState(gpio_pin)); }

// bool PID::operator=(const bool& value) {
//     GPIO_WritePin(gpio_pin, GPIO_PinState(value));
//     return value;
// }
// bool PID::operator!() {
//     GPIO_TogglePin(gpio_pin);
//     return this->operator bool();
// }
}  // namespace module