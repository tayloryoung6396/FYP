#include "Controller.hpp"
#include <iostream>
#include "utility/io/uart.hpp"

namespace module {
namespace Input {

    Controller controller = Controller(0, 0.1);

    Controller::Controller(uint32_t gpio, float length) : gpio(gpio), length(length) {}

    float Controller::GetPosition() {
        // Read gpio
        float position = ConvertPotentiometer(utility::io::adc_io.GetSensors(gpio));
        return position;
    }

    float Controller::ConvertPotentiometer(float position) {
        // Convert to angle based on potentiometer size
        return position;
    }
}  // namespace Input
}  // namespace module