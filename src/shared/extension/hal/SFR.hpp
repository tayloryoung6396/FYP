#ifndef EXTENSION_HAL_SFR_HPP
#define EXTENSION_HAL_SFR_HPP

#include <type_traits>
// #include "uart/CallbackStore.hpp"

namespace extension {
namespace hal {

    template <typename T, uint32_t address>
    struct SFR {
        static T* const reg;
    };

    template <typename T, uint32_t address>
    T* const SFR<T, address>::reg = reinterpret_cast<T* const>(address);

}  // namespace hal
}  // namespace extension

#endif  // EXTENSION_HAL_SFR_HPP