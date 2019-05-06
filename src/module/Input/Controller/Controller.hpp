#ifndef MODULE_CONTROLLER_HPP
#define MODULE_CONTROLLER_HPP

#include "Sensors/LinearPotentiometer/LinearPotentiometer.hpp"

namespace module {
namespace Input {

    struct input_t {
        Sensors::LinearPot& linear_pot;
    };

    class Controller {
    public:
        Controller();
    };

    extern Controller Controller1;
}  // namespace Input
}  // namespace module

#endif  // MODULE_CONTROLLER_HPP