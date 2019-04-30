#include "../../../nuclear/src/util/critical_section.hpp"
// #include "nuclear"
#include "stm32f7xx.h"

namespace NUClear {
namespace util {
    volatile int critical_section::count = 0;

    void critical_section::aquire_lock() { __disable_irq(); }

    void critical_section::release_lock() { __enable_irq(); }

}  // namespace util
}  // namespace NUClear
