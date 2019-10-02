#ifndef UTILITY_MATH_SATURATION_H
#define UTILITY_MATH_SATURATION_H

#include <algorithm>
#include <vector>

namespace utility {
namespace math {
    template <typename T>
    T sat(T theta, std::pair<T, T> limits) {
        if (theta > limits.first) {
            theta = limits.first;
        }
        else if (theta < limits.second) {
            theta = limits.second;
        }
        return (theta);
    }
}  // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_SATURATION_H
