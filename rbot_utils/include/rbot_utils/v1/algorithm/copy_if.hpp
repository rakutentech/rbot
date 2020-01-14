#ifndef _RBOT_UTILS_V1_ALGORITHM_COPY_IF_HPP_
#define _RBOT_UTILS_V1_ALGORITHM_COPY_IF_HPP_

namespace rbot_utils {
namespace v1 {
namespace detail {
template <class T, class U, class V>
constexpr void copy_if(T &destination, const U &origin, const V &condition)
{
    if (condition) {
        destination = origin;
    }
}
}  // namespace detail

template <class T, class U, class V>
constexpr void copy_1d_if(T &destination, const U &origin, const V &condition)
{
    detail::copy_if(destination.x, origin.x, condition.x);
}

template <class T, class U, class V>
constexpr void copy_2d_if(T &destination, const U &origin, const V &condition)
{
    copy_1d_if(destination, origin, condition);
    detail::copy_if(destination.y, origin.y, condition.y);
}

template <class T, class U, class V>
constexpr void copy_3d_if(T &destination, const U &origin, const V &condition)
{
    copy_2d_if(destination, origin, condition);
    detail::copy_if(destination.z, origin.z, condition.z);
}

template <class T, class U, class V>
constexpr void copy_4d_if(T &destination, const U &origin, const V &condition)
{
    copy_3d_if(destination, origin, condition);
    detail::copy_if(destination.w, origin.w, condition.w);
}

template <class T, class U, class V>
constexpr void copy_pose_if(T &destination, const U &origin, const V &condition)
{
    copy_3d_if(destination.position, origin.position, condition.position);
    copy_4d_if(
        destination.orientation, origin.orientation, condition.orientation);
}

template <class T, class U, class V>
constexpr void copy_motion_if(T &destination,
                              const U &origin,
                              const V &condition)
{
    copy_3d_if(destination.linear, origin.linear, condition.linear);
    copy_3d_if(destination.angular, origin.angular, condition.angular);
}

template <class T, class U, class V>
constexpr void copy_velocity_if(T &destination,
                                const U &origin,
                                const V &condition)
{
    copy_motion_if(destination, origin, condition);
}

template <class T, class U, class V>
constexpr void copy_acceleration_if(T &destination,
                                    const U &origin,
                                    const V &condition)
{
    copy_motion_if(destination, origin, condition);
}

template <class T, class U, class V>
constexpr void copy_setpoint_if(T &destination,
                                const U &origin,
                                const V &condition)
{
    copy_pose_if(destination.pose, origin.pose, condition.pose);
    copy_velocity_if(destination.velocity, origin.velocity, condition.velocity);
    copy_acceleration_if(
        destination.acceleration, origin.acceleration, condition.acceleration);
}
}  // namespace v1
}  // namespace rbot_utils
#endif  // _RBOT_UTILS_V1_ALGORITHM_COPY_IF_HPP_
