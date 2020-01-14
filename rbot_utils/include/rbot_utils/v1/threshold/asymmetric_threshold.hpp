#ifndef _RBOT_UTILS_V1_THRESHOLD_ASYMMETRIC_THRESHOLD_HPP_
#define _RBOT_UTILS_V1_THRESHOLD_ASYMMETRIC_THRESHOLD_HPP_

#include <cmath>

#include <rbot_utils/v1/math.hpp>

namespace rbot_utils {
namespace v1 {
template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void asymmetric_threshold_1d(Data& point_,
                                       CompareResult& result_,
                                       const LowerLimit& lower_threshold_,
                                       const UpperLimit& upper_threshold_,
                                       const Zero& zero_ = Zero())
{
    asymmetric_threshold_result(
        point_.x, result_.x, lower_threshold_.x, upper_threshold_.x, zero_.x);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void asymmetric_threshold_2d(Data& point_,
                                       CompareResult& result_,
                                       const LowerLimit& lower_threshold_,
                                       const UpperLimit& upper_threshold_,
                                       const Zero& zero_ = Zero())
{
    asymmetric_threshold_1d(
        point_, result_, lower_threshold_, upper_threshold_, zero_);
    asymmetric_threshold_result(
        point_.y, result_.y, lower_threshold_.y, upper_threshold_.y, zero_.y);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void asymmetric_threshold_3d(Data& point_,
                                       CompareResult& result_,
                                       const LowerLimit& lower_threshold_,
                                       const UpperLimit& upper_threshold_,
                                       const Zero& zero_ = Zero())
{
    asymmetric_threshold_2d(
        point_, result_, lower_threshold_, upper_threshold_, zero_);
    asymmetric_threshold_result(
        point_.z, result_.z, lower_threshold_.z, upper_threshold_.z, zero_.z);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void asymmetric_threshold_4d(Data& point_,
                                       CompareResult& result_,
                                       const LowerLimit& lower_threshold_,
                                       const UpperLimit& upper_threshold_,
                                       const Zero& zero_ = Zero())
{
    asymmetric_threshold_3d(
        point_, result_, lower_threshold_, upper_threshold_, zero_);
    asymmetric_threshold_result(
        point_.w, result_.w, lower_threshold_.w, upper_threshold_.w, zero_.w);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void asymmetric_threshold_pose(Data& point_,
                                         CompareResult& result_,
                                         const LowerLimit& lower_threshold_,
                                         const UpperLimit& upper_threshold_,
                                         const Zero& zero_ = Zero())
{
    asymmetric_threshold_3d(point_.position,
                            result_.position,
                            lower_threshold_.position,
                            upper_threshold_.position,
                            zero_.position);
    asymmetric_threshold_4d(point_.orientation,
                            result_.orientation,
                            lower_threshold_.orientation,
                            upper_threshold_.orientation,
                            zero_.orientation);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void asymmetric_threshold_motion(Data& point_,
                                           CompareResult& result_,
                                           const LowerLimit& lower_threshold_,
                                           const UpperLimit& upper_threshold_,
                                           const Zero& zero_ = Zero())
{
    asymmetric_threshold_3d(point_.linear,
                            result_.linear,
                            lower_threshold_.linear,
                            upper_threshold_.linear,
                            zero_.linear);
    asymmetric_threshold_4d(point_.angular,
                            result_.angular,
                            lower_threshold_.angular,
                            upper_threshold_.angular,
                            zero_.angular);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void asymmetric_threshold_velocity(Data& point_,
                                             CompareResult& result_,
                                             const LowerLimit& lower_threshold_,
                                             const UpperLimit& upper_threshold_,
                                             const Zero& zero_ = Zero())
{
    asymmetric_threshold_motion(
        point_, result_, lower_threshold_, upper_threshold_, zero_);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void asymmetric_threshold_acceleration(
    Data& point_,
    CompareResult& result_,
    const LowerLimit& lower_threshold_,
    const UpperLimit& upper_threshold_,
    const Zero& zero_ = Zero())
{
    asymmetric_threshold_motion(
        point_, result_, lower_threshold_, upper_threshold_, zero_);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void asymmetric_threshold_setpoint(Data& point_,
                                             CompareResult& result_,
                                             const LowerLimit& lower_threshold_,
                                             const UpperLimit& upper_threshold_,
                                             const Zero& zero_ = Zero())
{
    asymmetric_threshold_pose(point_.pose,
                              result_.pose,
                              lower_threshold_.pose,
                              upper_threshold_.pose,
                              zero_.pose);
    asymmetric_threshold_velocity(point_.velocity,
                                  result_.velocity,
                                  lower_threshold_.velocity,
                                  upper_threshold_.velocity,
                                  zero_.velocity);
    asymmetric_threshold_acceleration(point_.acceleration,
                                      result_.acceleration,
                                      lower_threshold_.acceleration,
                                      upper_threshold_.acceleration,
                                      zero_.acceleration);
}
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_THRESHOLD_ASYMMETRIC_THRESHOLD_HPP_HPP_ */
