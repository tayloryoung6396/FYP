#include "uart.hpp"
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
        char* string;
        if (0 < vsprintf(string, fmt, argp)) {
            HAL_UART_Transmit(&huart3, (uint8_t*) string, strlen(string), 0xFFFF);
        }
        va_end(argp);
    }
}  // namespace io
}  // namespace utility
