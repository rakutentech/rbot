#ifndef _RBOT_UTILS_V1_ALGORITHM_COPY_HPP_
#define _RBOT_UTILS_V1_ALGORITHM_COPY_HPP_

#include <utility>

#include <rbot_utils/v1/algorithm/helper_macro.hpp>
#include <rbot_utils/v1/type_traits.hpp>

namespace rbot_utils {
namespace v1 {
namespace detail {
namespace internal {
__FROM_LOWER_UPPER__(x, X);
__FROM_LOWER_UPPER__(y, Y);
__FROM_LOWER_UPPER__(z, Z);
__FROM_LOWER_UPPER__(w, W);

__TO_LOWER_UPPER__(x, X);
__TO_LOWER_UPPER__(y, Y);
__TO_LOWER_UPPER__(z, Z);
__TO_LOWER_UPPER__(w, W);
}  // namespace internal

__FROM_INTERNAL__(x);
__FROM_INTERNAL__(y);
__FROM_INTERNAL__(z);
__FROM_INTERNAL__(w);

__TO_INTERNAL__(x);
__TO_INTERNAL__(y);
__TO_INTERNAL__(z);
__TO_INTERNAL__(w);
}  // namespace detail
/*
 * memcpy not used because we don't know the order of the fields
 */
/**
 * @fn copy_1d
 * @brief copies x and y fields between 1 classes
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
constexpr void copy_1d(T &destination, const U &origin)
{
    detail::to_x(destination, detail::from_x(origin));
}

/**
 * @fn copy_1d_get
 * @brief sets x and y fields by calling X and Y getters
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
[[deprecated("Use copy_1d")]] constexpr void copy_1d_get(T &destination,
                                                         const U &origin)
{
    detail::to_x(destination, detail::from_x(origin));
}

/**
 * @fn copy_1d_set
 * @brief sets by calling X and Y setters from x and y fields
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
[[deprecated("Use copy_1d")]] constexpr void copy_1d_set(T &destination,
                                                         const U &origin)
{
    detail::to_x(destination, detail::from_x(origin));
}

/**
 * @fn copy_1d_set
 * @brief sets by calling X and Y setters from X and Y getters
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
[[deprecated("Use copy_1d")]] constexpr void copy_1d_get_set(T &destination,
                                                             const U &origin)
{
    detail::to_x(destination, detail::from_x(origin));
}

/**
 * @fn copy_2d
 * @brief copies x and y fields between 2 classes
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
constexpr void copy_2d(T &destination, const U &origin)
{
    copy_1d(destination, origin);
    detail::to_y(destination, detail::from_y(origin));
}

/**
 * @fn copy_2d_get
 * @brief sets x and y fields by calling X and Y getters
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
[[deprecated("Use copy_2d")]] constexpr void copy_2d_get(T &destination,
                                                         const U &origin)
{
    copy_1d(destination, origin);
    detail::to_y(destination, detail::from_y(origin));
}

/**
 * @fn copy_2d_set
 * @brief sets by calling X and Y setters from x and y fields
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
[[deprecated("Use copy_2d")]] constexpr void copy_2d_set(T &destination,
                                                         const U &origin)
{
    copy_1d(destination, origin);
    detail::to_y(destination, detail::from_y(origin));
}

/**
 * @fn copy_2d_set
 * @brief sets by calling X and Y setters from X and Y getters
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
[[deprecated("Use copy_2d")]] constexpr void copy_2d_get_set(T &destination,
                                                             const U &origin)
{
    copy_1d(destination, origin);
    detail::to_y(destination, detail::from_y(origin));
}

/**
 * @fn copy_3
 * @brief copies x, y and z fields between 2 classes
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
constexpr void copy_3d(T &destination, const U &origin)
{
    copy_2d(destination, origin);
    detail::to_z(destination, detail::from_z(origin));
}

/**
 * @fn copy_3d_get
 * @brief sets x and y fields by calling X and Y getters
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
[[deprecated("Use copy_3d")]] constexpr void copy_3d_get(T &destination,
                                                         const U &origin)
{
    copy_2d(destination, origin);
    detail::to_z(destination, detail::from_z(origin));
}

/**
 * @fn copy_3d_set
 * @brief sets by calling X, Y and Z setters from x, y and z fields
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
[[deprecated("Use copy_3d")]] constexpr void copy_3d_set(T &destination,
                                                         const U &origin)
{
    copy_2d(destination, origin);
    detail::to_z(destination, detail::from_z(origin));
}

/**
 * @fn copy_3d_set
 * @brief sets by calling X, Y and Z setters from X, Y and Z getters
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
[[deprecated("Use copy_3d")]] constexpr void copy_3d_get_set(T &destination,
                                                             const U &origin)
{
    copy_2d(destination, origin);
    detail::to_z(destination, detail::from_z(origin));
}

/**
 * @fn copy_4d
 * @brief copies x, y, z and w fields between 2 classes
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
constexpr void copy_4d(T &destination, const U &origin)
{
    copy_3d(destination, origin);
    detail::to_w(destination, detail::from_w(origin));
}

/**
 * @fn copy_4d_get
 * @brief sets x and y fields by calling X and Y getters
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
[[deprecated("Use copy_4d")]] constexpr void copy_4d_get(T &destination,
                                                         const U &origin)
{
    copy_3d(destination, origin);
    detail::to_w(destination, detail::from_w(origin));
}

/**
 * @fn copy_3d_set
 * @brief sets by calling X, Y, Z and W setters from x, y, z and w fields
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
[[deprecated("Use copy_4d")]] constexpr void copy_4d_set(T &destination,
                                                         const U &origin)
{
    copy_3d(destination, origin);
    detail::to_w(destination, detail::from_w(origin));
}

/**
 * @fn copy_3d_get_set
 * @brief sets by calling X, Y, Z and W setters from X, Y, Z and W getters
 * @param[out] destination tar_get
 * @param[in] origin source
 */
template <class T, class U>
[[deprecated("Use copy_4d")]] constexpr void copy_4d_get_set(T &destination,
                                                             const U &origin)
{
    copy_3d(destination, origin);
    detail::to_w(destination, detail::from_w(origin));
}

template <class T, class U>
constexpr void copy_pose(T &destination, const U &origin)
{
    copy_3d(destination.position, origin.position);
    copy_4d(destination.orientation, origin.orientation);
}

template <class T, class U>
constexpr void copy_motion(T &destination, const U &origin)
{
    copy_3d(destination.linear, origin.linear);
    copy_3d(destination.angular, origin.angular);
}

template <class T, class U>
constexpr void copy_velocity(T &destination, const U &origin)
{
    copy_motion(destination, origin);
}

template <class T, class U>
constexpr void copy_acceleration(T &destination, const U &origin)
{
    copy_motion(destination, origin);
}

template <class T, class U>
constexpr void copy_setpoint(T &destination, const U &origin)
{
    copy_pose(destination.pose, origin.pose);
    copy_velocity(destination.velocity, origin.velocity);
    copy_acceleration(destination.acceleration, origin.acceleration);
}
}  // namespace v1
}  // namespace rbot_utils
#endif  // _RBOT_UTILS_V1_ALGORITHM_COPY_HPP_
