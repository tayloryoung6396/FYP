#include "Valve.hpp"

namespace module {
namespace HardwareIO {

    Valve valve1 = Valve(1);
    Valve valve2 = Valve(2);

    Valve::Valve(uint32_t gpio) {}

    // // Get the pin state from the MCU for the pin and return it
    // Valve::operator bool() { return bool(true); }

    // bool Valve::operator=(const bool& value) { return value; }
    // bool Valve::operator!() { return this->operator bool(); }
}  // namespace HardwareIO
}  // namespace module