#ifndef _RBOT_UTILS_V1_ALGORITHM_ALL_HPP_
#define _RBOT_UTILS_V1_ALGORITHM_ALL_HPP_

namespace rbot_utils {
namespace v1 {
template <class T>
constexpr bool all_1d(T &origin)
{
    return origin.x;
}

template <class T>
constexpr bool all_2d(T &origin)
{
    return origin.y && all_1d(origin);
}

template <class T>
constexpr bool all_3d(T &origin)
{
    return origin.z && all_2d(origin);
}

template <class T>
constexpr bool all_4d(T &origin)
{
    return origin.w && all_3d(origin);
}

template <class T>
constexpr bool all_pose(T &origin)
{
    return all_3d(origin.position) && all_4d(origin.orientation);
}

template <class T>
constexpr bool all_motion(T &origin)
{
    return all_3d(origin.linear) && all_3d(origin.angular);
}

template <class T>
constexpr bool all_velocity(T &origin)
{
    return all_motion(origin);
}

template <class T>
constexpr bool all_acceleration(T &origin)
{
    return all_motion(origin);
}

template <class T>
constexpr bool all_setpoint(T &origin)
{
    return all_pose(origin.pose) && all_velocity(origin.velocity) &&
           all_acceleration(origin.acceleration);
}
}  // namespace v1
}  // namespace rbot_utils
#endif  // _RBOT_UTILS_V1_ALGORITHM_ALL_HPP_
