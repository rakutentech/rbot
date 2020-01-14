#ifndef _RBOT_UTILS_V1_MATH_ANGLES_HPP_
#define _RBOT_UTILS_V1_MATH_ANGLES_HPP_

#include <cmath>
#include <type_traits>
#include <utility>

#if __cplusplus < 201703L
#include <boost/optional.hpp>
namespace std {
template <class T>
using optional = boost::optional<T>;
}
#endif

namespace rbot_utils {
namespace v1 {
template <class T>
struct is_floating_point : std::is_floating_point<T>
{};

constexpr double pi = M_PI;

/**
 * @brief degree to radians, no limits
 */
template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
constexpr T deg2rad(const T&& degree)
{
    return degree * pi / 180.;
}

/**
 * @brief radians to degree, no limits
 */
template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
constexpr T rad2deg(const T&& radians)
{
    return radians * 180. / pi;
}
template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
constexpr T rad2deg(const T& radians)
{
    return radians * 180. / pi;
}

/**
 * @brief radians [-inf, inf] -> [-2*pi, 2*pi]
 */
template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
constexpr T normalize_angle_direction(const T&& radians)
{
    return std::fmod(radians, 2 * pi);
}
template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
constexpr T normalize_angle_direction(const T& radians)
{
    return std::fmod(radians, 2 * pi);
}

/**
 * @brief radians [-inf, inf] -> [0, 2*pi)
 */
template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
constexpr T normalize_angle_positive(const T&& radians)
{
    auto angle = normalize_angle_direction(radians);
    // comparison with branches is slower than multiplication with bool
    // T might be unsigned, in which case comparison < 0 is undefined behavior
    return angle + (!(0 <= angle)) * 2 * pi;
}
template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
constexpr T normalize_angle_positive(const T& radians)
{
    auto angle = normalize_angle_direction(radians);
    return angle + (!(0 <= angle)) * 2 * pi;
}

/**
 * @brief radians [-inf, inf] -> (-pi, pi]
 */
template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
constexpr T normalize_angle(const T& radians)
{
    auto angle = normalize_angle_positive(radians);
    return angle - (angle > pi) * 2 * pi;
}

template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
constexpr T two_pi_compliment(const T&& radians)
{
    auto angle = normalize_angle_direction(radians);
    return angle + std::copysign(2 * pi, angle);
}

template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
struct PlaceholderName
{
    PlaceholderName() = default;
    PlaceholderName(T clockwise_, T counterclockwise_)
        : clockwise(clockwise_), counterclockwise(counterclockwise_)
    {}
    T clockwise;
    T counterclockwise;
};

template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
constexpr PlaceholderName<std::optional<T>> find_safety_margin(
    const T&& from_radians, const PlaceholderName<T>&& limits_radians)
{
    PlaceholderName<std::optional<T>> safety_margin;

    PlaceholderName<T> diff(limits_radians.clockwise - from_radians,
                            limits_radians.counterclockwise - from_radians);

    if (diff.clockwise >= 0) {
        safety_margin.clockwise.emplace(diff.clockwise);
    }
    if (diff.counterclockwise <= 0) {
        safety_margin.counterclockwise.emplace(diff.counterclockwise);
    }
    return safety_margin;
}

template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
constexpr T shortest_angular_distance(const T&& from_radians,
                                      const T&& to_radians)
{
    return normalize_angle(to_radians - from_radians);
}

template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
constexpr T second_shortest_angular_distance(const T&& from_radians,
                                             const T&& to_radians)
{
    return two_pi_compliment(to_radians - from_radians);
}

template <class T, class = std::enable_if_t<is_floating_point<T>::value>>
constexpr PlaceholderName<std::optional<T>> shortest_angular_distance(
    const T&& from_radians,
    const T&& to_radians,
    const PlaceholderName<T>&& limits_radians,
    bool balance = false)
{
    auto limits = find_safety_margin(from_radians, limits_radians);
    assert(false);
}
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_MATH_ANGLES_HPP_ */
