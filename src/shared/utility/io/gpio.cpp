#include "gpio.hpp"

namespace utility {
namespace io {
    namespace gpio {

        GPIO valve[12] = {GPIO(GPIOE, GPIO_PIN_2),
                          GPIO(GPIOE, GPIO_PIN_4),
                          GPIO(GPIOE, GPIO_PIN_5),
                          GPIO(GPIOE, GPIO_PIN_10),
                          GPIO(GPIOE, GPIO_PIN_12),
                          GPIO(GPIOE, GPIO_PIN_13),
                          GPIO(GPIOE, GPIO_PIN_14),
                          GPIO(GPIOE, GPIO_PIN_15),
                          GPIO(GPIOF, GPIO_PIN_2),
                          GPIO(GPIOG, GPIO_PIN_1),
                          GPIO(GPIOG, GPIO_PIN_3),
                          GPIO(GPIOD, GPIO_PIN_11)};

        GPIO led3 = GPIO(GPIOB, LD3_Pin);
        GPIO led2 = GPIO(GPIOB, LD2_Pin);


    };  // namespace gpio

    GPIO::GPIO(GPIO_TypeDef* gpio_bank, uint16_t gpio_pin) : gpio_bank(gpio_bank), gpio_pin(gpio_pin) {}

    GPIO::operator bool() { return bool(HAL_GPIO_ReadPin(gpio_bank, gpio_pin)); }

    bool GPIO::operator=(const bool& value) {
        HAL_GPIO_WritePin(gpio_bank, gpio_pin, GPIO_PinState(value));
        return value;
    }
    bool GPIO::operator!() {
        HAL_GPIO_TogglePin(gpio_bank, gpio_pin);
        return this->operator bool();
    }

}  // namespace io
}  // namespace utility
