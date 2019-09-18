#include "gpio.hpp"
#include "gpio.h"

extern "C" {
void MX_GPIO_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    HAL_GPIO_WritePin(
        GPIOE,
        GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
        GPIO_PIN_RESET);

    // Configure GPIO pin Output Level
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);

    // Configure GPIO pin Output Level
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_RESET);

    // Configure GPIO pin Output Level
    HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

    // Configure GPIO pin Output Level
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);

    // Configure GPIO pins : PE2 PE4 PE5 PE10 PE12 PE13 PE14 PE15
    GPIO_InitStruct.Pin =
        GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // Configure GPIO pin : PF2
    GPIO_InitStruct.Pin   = GPIO_PIN_2;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    // Configure GPIO pins : PG1 PG3
    GPIO_InitStruct.Pin   = GPIO_PIN_1 | GPIO_PIN_3;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    // Configure GPIO pins : PBPin PBPin
    GPIO_InitStruct.Pin   = LD3_Pin | LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure GPIO pin : PD11
    GPIO_InitStruct.Pin   = GPIO_PIN_11;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


    // Configure GPIO pins : PA8 PA10 PA11 PA12
    // GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
}

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
