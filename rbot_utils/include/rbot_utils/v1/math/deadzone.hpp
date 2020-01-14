#ifndef _RBOT_UTILS_V1_MATH_DEADZONE_HPP_
#define _RBOT_UTILS_V1_MATH_DEADZONE_HPP_

#include <cmath>
#include <utility>

#include <rbot_utils/v1/math/signum.hpp>

namespace rbot_utils {
namespace v1 {
template <class T>
constexpr T deadzone_branched(
    const T& value,
    const T& threshold,
    const T& zero =
        T()) noexcept(noexcept(std::declval<T>() - std::declval<T>()) &&
                      noexcept(std::declval<T>() < std::declval<T>()))
{
    if (abs(value - zero) < abs(threshold)) {
        return zero;
    }
    return value;
}
template <class T>
constexpr T deadzone_branchless(
    const T& value,
    const T& threshold,
    const T& zero =
        T()) noexcept(noexcept(std::declval<T>() - std::declval<T>()) &&
                      noexcept(std::declval<T>() < std::declval<T>()) &&
                      noexcept(std::declval<T>() * std::declval<T>()))
{
    return (!(abs(value - zero) < abs(threshold))) * value;
}

template <class T>
constexpr T deadzone(
    const T& value,
    const T& threshold,
    const T& zero = T()) noexcept(noexcept(deadzone_branched(std::declval<T>(),
                                                            std::declval<T>(),
                                                            std::declval<T>())))
{
    return deadzone_branched(value, threshold, zero);
}

static_assert(1 == deadzone(4, 6, 1));
static_assert(1.3 == deadzone(4.1, 6.2, 1.3));
static_assert(4 == deadzone(4, 2, 1));
static_assert(4.1 == deadzone(4.1, 2.2, 1.3));
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_MATH_DEADZONE_HPP_ */
