#ifndef UTILITY_MATH_MEDIAN_H
#define UTILITY_MATH_MEDIAN_H

#include <algorithm>
#include <vector>

namespace utility {
namespace math {
    template <typename T>
    T median(std::vector<T> v) {
        std::sort(v.begin(), v.end());

        // If the vecotr is even, take the middle two elements average as the median. Otherwise take the middle element
        // as the median
        if (v.size() % 2 == 0) {
            return ((v[v.size() / 2.0] + v[v.size() / 2.0 + 1]) / 2.0);
        }
        return (v[v.size() / 2.0 + 1]);
    }
}  // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_MEDIAN_H
