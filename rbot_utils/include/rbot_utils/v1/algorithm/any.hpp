#ifndef _RBOT_UTILS_V1_ALGORITHM_ANY_HPP_
#define _RBOT_UTILS_V1_ALGORITHM_ANY_HPP_

namespace rbot_utils {
namespace v1 {
template <class T>
constexpr bool any_1d(T &origin)
{
    return origin.x;
}

template <class T>
constexpr bool any_2d(T &origin)
{
    return origin.y || any_1d(origin);
}

template <class T>
constexpr bool any_3d(T &origin)
{
    return origin.z || any_2d(origin);
}

template <class T>
constexpr bool any_4d(T &origin)
{
    return origin.w || any_3d(origin);
}

template <class T>
constexpr bool any_pose(T &origin)
{
    return any_3d(origin.position) || any_4d(origin.orientation);
}

template <class T>
constexpr bool any_motion(T &origin)
{
    return any_3d(origin.linear) || any_3d(origin.angular);
}

template <class T>
constexpr bool any_velocity(T &origin)
{
    return any_motion(origin);
}

template <class T>
constexpr bool any_acceleration(T &origin)
{
    return any_motion(origin);
}

template <class T>
constexpr bool any_setpoint(T &origin)
{
    return any_pose(origin.pose) || any_velocity(origin.velocity) ||
           any_acceleration(origin.acceleration);
}
}  // namespace v1
}  // namespace rbot_utils
#endif  // _RBOT_UTILS_V1_ALGORITHM_ANY_HPP_
