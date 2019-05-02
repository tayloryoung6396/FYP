#ifndef UTILITY_IO_UART_HPP
#define UTILITY_IO_UART_HPP

#include <stdarg.h>
#include <stdint.h>
#include "stm32f7xx.h"

namespace utility {
namespace io {

    class UART {
    public:
        UART();

        void out(const char* fmt, ...);
    };

    extern UART debug;

}  // namespace io
}  // namespace utility

#endif  // UTILITY_IO_UART_HPP
