#ifndef _RBOT_UTILS_V1_ROS_CLOCK_HPP_
#define _RBOT_UTILS_V1_ROS_CLOCK_HPP_

#include <ros/time.h>

namespace rbot_utils {
namespace v1 {
struct ros_clock : public ros::Time
{
    using rep = double;
    using period = std::ratio<1, 1>;
    using duration = ros::Duration;
    using time_point = ros::Time;
    constexpr static bool is_steady(void) noexcept { return false; }
    static time_point now(void) noexcept(noexcept(ros::Time::now()))
    {
        return ros::Time::now();
    }
};
}  // namespace v1
}  // namespace rbot_utils

#endif /* ifndef _RBOT_UTILS_V1_ROS_CLOCK_HPP_ */
