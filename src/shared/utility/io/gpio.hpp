#ifndef UTILITY_IO_GPIO_HPP
#define UTILITY_IO_GPIO_HPP
#include "gpio.h"

namespace utility {
namespace io {

    class GPIO {
    private:
        GPIO_TypeDef* gpio_bank;
        uint16_t gpio_pin;

    public:
        GPIO(GPIO_TypeDef* gpio_bank, uint16_t gpio_pin);

        operator bool();
        bool operator=(const bool& value);
        bool operator!();
    };

    namespace gpio {
        extern GPIO valve[12];
        extern GPIO led3;
        extern GPIO led2;
    }  // namespace gpio
}  // namespace io
}  // namespace utility
#endif  // UTILITY_IO_GPIO_HPP
