#ifndef MODULE_CONTROLLER_HPP
#define MODULE_CONTROLLER_HPP

#include <stdint.h>
#include "Sensors/LinearPotentiometer/LinearPotentiometer.hpp"
#include "utility/io/adc.hpp"

namespace module {
namespace Input {

    class Controller {
    public:
        Controller(uint32_t gpio, float length);

        float GetPosition();

        float ConvertPotentiometer(float position);

    private:
        uint32_t gpio;

        extern Controller Controller1;
    }  // namespace Input
}  // namespace Input