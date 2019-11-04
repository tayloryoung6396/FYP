#ifndef UTILITY_IO_SIMULATED_UART_HPP
#define UTILITY_IO_SIMULATED_UART_HPP

#include <stdint.h>
#include <cstdarg>
#include <cstdio>
#include "stm32f7xx.h"

namespace utility {
namespace io {

    class UART {
    public:
        UART();

        void out(const char* fmt, ...);
        void error(const char* fmt, ...);
        void info(const char* fmt, ...);
    };

    extern UART debug;

}  // namespace io
}  // namespace utility

#endif  // UTILITY_IO_SIMULATED_UART_HPP
