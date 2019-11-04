#include "simulated_uart.hpp"
#include <cstdio>
#include <cstring>

#define KNRM "\e[0m"
#define KRED "\e[31m"
#define KGRN "\e[32m"
#define KYEL "\e[33m"
#define KBLU "\e[34m"
#define KMAG "\e[35m"
#define KCYN "\e[36m"
#define KWHT "\e[37m"

#define CLEAR_M "\014"
#define RESET_M "\033[3J"
#define DEFAULT_COLOUR_M KNRM
#define DEBUG_M KYEL
#define SYS_M KYEL
#define ERROR_M KRED

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

    void UART::error(const char* fmt, ...) {
        va_list argp;
        va_start(argp, fmt);
        char msg[256]  = {0};
        char msg2[256] = {0};
        int n          = sprintf(msg2, "%sERROR:%s", ERROR_M, DEFAULT_COLOUR_M);
        if (0 < vsprintf((char*) msg, fmt, argp)) {
            HAL_UART_Transmit(&huart6, (uint8_t*) &msg2, strlen((char*) msg), 0xFFFF);
            HAL_UART_Transmit(&huart6, (uint8_t*) &msg, strlen((char*) msg), 0xFFFF);
        }
        va_end(argp);
    }

    void UART::info(const char* fmt, ...) {
        va_list argp;
        va_start(argp, fmt);
        char msg[256]  = {0};
        char msg2[256] = {0};
        int n          = sprintf(msg2, "%sINFO:%s", SYS_M, DEFAULT_COLOUR_M);
        if (0 < vsprintf((char*) msg, fmt, argp)) {
            HAL_UART_Transmit(&huart6, (uint8_t*) &msg2, strlen((char*) msg), 0xFFFF);
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