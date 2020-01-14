#ifndef _RBOT_UTILS_V1_MATH_SIGNUM_HPP_
#define _RBOT_UTILS_V1_MATH_SIGNUM_HPP_

#include <cmath>
#include <type_traits>

namespace rbot_utils {
namespace v1 {
// copysign isn't sufficient because -0 and +0 have different outputs
template <class T>
constexpr auto signum(const T& x)
    -> std::enable_if_t<std::is_signed<T>::value, int>
{
    return (T(0) < x) - (x < T(0));
}
template <class T>
constexpr auto signum(const T& x)
    -> std::enable_if_t<!std::is_signed<T>::value, int>
{
    return T(0) < x;
}

template <class T>
constexpr T abs(const T& value)
{
    return value * signum(value);
}

static_assert(1 == signum(23));
static_assert(-1 == signum(-23));
static_assert(1 == signum(23.5));
static_assert(-1 == signum(-23.5));

static_assert(23 == abs(23));
static_assert(23 == abs(-23));
static_assert(23.5 == abs(23.5));
static_assert(23.5 == abs(-23.5));
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_MATH_SIGNUM_HPP_ */
