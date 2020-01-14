#ifndef _RBOT_UTILS_V1_THRESHOLD_LIMIT_HPP_
#define _RBOT_UTILS_V1_THRESHOLD_LIMIT_HPP_

#include <cmath>

#include <rbot_utils/v1/math.hpp>

namespace rbot_utils {
namespace v1 {
template <class Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class CompareResult = Data,
          class Zero = Data>
constexpr void limit_1d(Data& point_,
                        CompareResult& result_,
                        const LowerLimit& lower_limit_,
                        const UpperLimit& upper_limit_,
                        const Zero& zero_ = Zero())
{
    limit(point_.x, result_.x, lower_limit_.x, upper_limit_.x, zero_.x);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void limit_2d(Data& point_,
                        CompareResult& result_,
                        const LowerLimit& lower_limit_,
                        const UpperLimit& upper_limit_,
                        const Zero& zero_ = Zero())
{
    limit_1d(point_, result_, lower_limit_, upper_limit_, zero_);
    limit(point_.y, result_.y, lower_limit_.y, upper_limit_.y, zero_.y);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void limit_3d(Data& point_,
                        CompareResult& result_,
                        const LowerLimit& lower_limit_,
                        const UpperLimit& upper_limit_,
                        const Zero& zero_ = Zero())
{
    limit_2d(point_, result_, lower_limit_, upper_limit_, zero_);
    limit(point_.z, result_.z, lower_limit_.z, upper_limit_.z, zero_.z);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void limit_4d(Data& point_,
                        CompareResult& result_,
                        const LowerLimit& lower_limit_,
                        const UpperLimit& upper_limit_,
                        const Zero& zero_ = Zero())
{
    limit_3d(point_, result_, lower_limit_, upper_limit_, zero_);
    limit(point_.w, result_.w, lower_limit_.w, upper_limit_.w, zero_.w);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void limit_pose(Data& point_,
                          CompareResult& result_,
                          const LowerLimit& lower_limit_,
                          const UpperLimit& upper_limit_,
                          const Zero& zero_ = Zero())
{
    limit_3d(point_.position,
             result_.position,
             lower_limit_.position,
             upper_limit_.position,
             zero_.position);
    limit_4d(point_.orientation,
             result_.orientation,
             lower_limit_.orientation,
             upper_limit_.orientation,
             zero_.orientation);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void limit_motion(Data& point_,
                            CompareResult& result_,
                            const LowerLimit& lower_limit_,
                            const UpperLimit& upper_limit_,
                            const Zero& zero_ = Zero())
{
    limit_3d(point_.linear,
             result_.linear,
             lower_limit_.linear,
             upper_limit_.linear,
             zero_.linear);
    limit_4d(point_.angular,
             result_.angular,
             lower_limit_.angular,
             upper_limit_.angular,
             zero_.angular);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void limit_velocity(Data& point_,
                              CompareResult& result_,
                              const LowerLimit& lower_limit_,
                              const UpperLimit& upper_limit_,
                              const Zero& zero_ = Zero())
{
    limit_motion(point_, result_, lower_limit_, upper_limit_, zero_);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void limit_acceleration(Data& point_,
                                  CompareResult& result_,
                                  const LowerLimit& lower_limit_,
                                  const UpperLimit& upper_limit_,
                                  const Zero& zero_ = Zero())
{
    limit_motion(point_, result_, lower_limit_, upper_limit_, zero_);
}

template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data>
constexpr void limit_setpoint(Data& point_,
                              CompareResult& result_,
                              const LowerLimit& lower_limit_,
                              const UpperLimit& upper_limit_,
                              const Zero& zero_ = Zero())
{
    limit_pose(point_.pose,
               result_.pose,
               lower_limit_.pose,
               upper_limit_.pose,
               zero_.pose);
    limit_velocity(point_.velocity,
                   result_.velocity,
                   lower_limit_.velocity,
                   upper_limit_.velocity,
                   zero_.velocity);
    limit_acceleration(point_.acceleration,
                       result_.acceleration,
                       lower_limit_.acceleration,
                       upper_limit_.acceleration,
                       zero_.acceleration);
}
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_THRESHOLD_LIMIT_HPP_HPP_ */
