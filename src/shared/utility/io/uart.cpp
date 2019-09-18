#include "uart.hpp"
#include <cstdio>
#include <cstring>
#include "stm32f7xx.h"
#include "usart.h"

namespace utility {
namespace io {
    UART debug = UART();

    UART::UART() {}

    void UART::out(const char* fmt, ...) {
        va_list argp;
        va_start(argp, fmt);
        char msg[256] = {0};
        if (0 < vsprintf((char*) msg, fmt, argp)) {
            HAL_UART_Transmit(&huart6, (uint8_t*) &msg, strlen((char*) msg), 0xFFFF);
        }
        va_end(argp);
    }
}  // namespace io
}  // namespace utility

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart6, (uint8_t*) (&ch), 1, HAL_MAX_DELAY);
    return 1;
}

int __io_getchar(void) {
    uint8_t ch;
    HAL_UART_Receive(&huart6, (uint8_t*) (&ch), 1, HAL_MAX_DELAY);
    return (int) ch;
}
