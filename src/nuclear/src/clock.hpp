#ifndef NUCLEAR_CLOCK_HPP
#define NUCLEAR_CLOCK_HPP

#include <chrono>

namespace NUClear {

struct clock {
    using rep                       = uint32_t;
    using period                    = std::ratio<1, 1000000>;
    using duration                  = std::chrono::duration<rep, period>;
    using time_point                = std::chrono::time_point<NUClear::clock>;
    static constexpr bool is_steady = true;

    static time_point now();

    static uint64_t clock_overflow;
};

struct timer {
    using rep                       = uint32_t;
    using period                    = std::ratio<1, 1000000>;
    using duration                  = std::chrono::duration<rep, period>;
    using time_point                = std::chrono::time_point<NUClear::clock>;
    static constexpr bool is_steady = true;

    static void start(const NUClear::clock::duration& duration);
};

}  // namespace NUClear

namespace utility {
namespace clock {
    void initialise();
}
}  // namespace utility
#endif  // NUCLEAR_CLOCK_HPP