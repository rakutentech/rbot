#ifndef _RBOT_UTILS_V1_THRESHOLD_SYMMETRIC_THRESHOLD_HPP_
#define _RBOT_UTILS_V1_THRESHOLD_SYMMETRIC_THRESHOLD_HPP_

#include <cmath>

#include <rbot_utils/v1/math.hpp>

namespace rbot_utils {
namespace v1 {
template <class Data,
          class CompareResult = Data,
          class Limit = Data,
          class Zero = Data>
constexpr void symmetric_threshold_1d(Data& point_,
                                      CompareResult& result_,
                                      const Limit& threshold_,
                                      const Zero& zero_ = Zero())
{
    symmetric_threshold_result(point_.x, result_.x, threshold_.x, zero_.x);
}

template <class Data,
          class CompareResult = Data,
          class Limit = Data,
          class Zero = Data>
constexpr void symmetric_threshold_2d(Data& point_,
                                      CompareResult& result_,
                                      const Limit& threshold_,
                                      const Zero& zero_ = Zero())
{
    symmetric_threshold_1d(point_, result_, threshold_, zero_);
    symmetric_threshold_result(point_.y, result_.y, threshold_.y, zero_.y);
}

template <class Data,
          class CompareResult = Data,
          class Limit = Data,
          class Zero = Data>
constexpr void symmetric_threshold_3d(Data& point_,
                                      CompareResult& result_,
                                      const Limit& threshold_,
                                      const Zero& zero_ = Zero())
{
    symmetric_threshold_2d(point_, result_, threshold_, zero_);
    symmetric_threshold_result(point_.z, result_.z, threshold_.z, zero_.z);
}

template <class Data,
          class CompareResult = Data,
          class Limit = Data,
          class Zero = Data>
constexpr void symmetric_threshold_4d(Data& point_,
                                      CompareResult& result_,
                                      const Limit& threshold_,
                                      const Zero& zero_ = Zero())
{
    symmetric_threshold_3d(point_, result_, threshold_, zero_);
    symmetric_threshold_result(point_.w, result_.w, threshold_.w, zero_.w);
}

template <class Data,
          class CompareResult = Data,
          class Limit = Data,
          class Zero = Data>
constexpr void symmetric_threshold_pose(Data& point_,
                                        CompareResult& result_,
                                        const Limit& threshold_,
                                        const Zero& zero_ = Zero())
{
    symmetric_threshold_3d(
        point_.position, result_.position, threshold_.position, zero_.position);
    symmetric_threshold_4d(point_.orientation,
                           result_.orientation,
                           threshold_.orientation,
                           zero_.orientation);
}

template <class Data,
          class CompareResult = Data,
          class Limit = Data,
          class Zero = Data>
constexpr void symmetric_threshold_motion(Data& point_,
                                          CompareResult& result_,
                                          const Limit& threshold_,
                                          const Zero& zero_ = Zero())
{
    symmetric_threshold_3d(
        point_.linear, result_.linear, threshold_.linear, zero_.linear);
    symmetric_threshold_3d(
        point_.angular, result_.angular, threshold_.angular, zero_.angular);
}

template <class Data,
          class CompareResult = Data,
          class Limit = Data,
          class Zero = Data>
constexpr void symmetric_threshold_velocity(Data& point_,
                                            CompareResult& result_,
                                            const Limit& threshold_,
                                            const Zero& zero_ = Zero())
{
    symmetric_threshold_motion(point_, result_, threshold_, zero_);
}

template <class Data,
          class CompareResult = Data,
          class Limit = Data,
          class Zero = Data>
constexpr void symmetric_threshold_acceleration(Data& point_,
                                                CompareResult& result_,
                                                const Limit& threshold_,
                                                const Zero& zero_ = Zero())
{
    symmetric_threshold_motion(point_, result_, threshold_, zero_);
}

template <class Data,
          class CompareResult = Data,
          class Limit = Data,
          class Zero = Data>
constexpr void symmetric_threshold_setpoint(Data& point_,
                                            CompareResult& result_,
                                            const Limit& threshold_,
                                            const Zero& zero_ = Zero())
{
    symmetric_threshold_pose(
        point_.pose, result_.pose, threshold_.pose, zero_.pose);
    symmetric_threshold_velocity(
        point_.velocity, result_.velocity, threshold_.velocity, zero_.velocity);
    symmetric_threshold_acceleration(point_.acceleration,
                                     result_.acceleration,
                                     threshold_.acceleration,
                                     zero_.acceleration);
}
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_THRESHOLD_SYMMETRIC_THRESHOLD_HPP_HPP_ */
