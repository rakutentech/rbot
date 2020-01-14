#ifndef _RBOT_UTILS_V1_MATH_THRESHOLD_HPP_
#define _RBOT_UTILS_V1_MATH_THRESHOLD_HPP_

#include <cmath>

#include <rbot_utils/v1/math/signum.hpp>

namespace rbot_utils {
namespace v1 {
template <class Data, class = void>
struct MinMax
{
    template <class Compare = std::less<>>
    constexpr decltype(auto) min(const Data& a,
                                 const Data& b,
                                 Compare compare_ = Compare())
    {
        return std::min(a, b, compare_);
    }
    template <class Compare = std::less<>>
    constexpr decltype(auto) max(const Data& a,
                                 const Data& b,
                                 Compare compare_ = Compare())
    {
        return std::max(a, b, compare_);
    }
};
}  // namespace v1
}  // namespace rbot_utils
#ifdef EIGEN_WORLD_VERSION
#include <Eigen/Dense>
namespace rbot_utils {
namespace v1 {
template <>
struct MinMax<Eigen::Array4f>
{
    using Data = Eigen::Array4f;
    template <class Compare = std::less<>>
    constexpr decltype(auto) min(const Data& a, const Data& b)
    {
        return a.min(b);
    }
    template <class Compare = std::less<>>
    constexpr decltype(auto) min(const Data& a, const Data& b, Compare compare_)
    {
        return min(a, b);
    }
    template <class Compare = std::less<>>
    constexpr decltype(auto) max(const Data& a, const Data& b)
    {
        return a.max(b);
    }
    template <class Compare = std::less<>>
    constexpr decltype(auto) max(const Data& a, const Data& b, Compare compare_)
    {
        return max(a, b);
    }
};
}  // namespace v1
}  // namespace rbot_utils
#endif

namespace rbot_utils {
namespace v1 {
template <class Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Compare = MinMax<Data>>
constexpr void limit(Data& point_,
                     const LowerLimit& lower_limit_,
                     const UpperLimit& upper_limit_,
                     Compare compare_ = Compare())
{
    point_ = compare_.min(upper_limit_, compare_.max(lower_limit_, point_));
}
template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Compare = MinMax<Data>>
constexpr void limit(Data& point_,
                     CompareResult& result_,
                     const LowerLimit& lower_limit_,
                     const UpperLimit& upper_limit_,
                     Compare compare_ = Compare())
{
    auto tmp = point_;
    limit(point_, lower_limit_, upper_limit_, compare_);
    result_ = signum(tmp - point_);
}
template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Compare = MinMax<Data>>
constexpr void limit_result(Data& point_,
                            CompareResult& result_,
                            const LowerLimit& lower_limit_,
                            const UpperLimit& upper_limit_,
                            Compare compare_ = Compare())
{
    limit(point_, result_, lower_limit_, upper_limit_, compare_);
}

template <class Data,
          class Limit = Data,
          class Zero = Data,
          class Compare = MinMax<Data>>
constexpr void symmetric_threshold(Data& point_,
                                   const Limit& threshold_,
                                   const Zero& zero_ = Zero(),
                                   Compare compare_ = Compare())
{
    limit(point_, zero_ - threshold_, zero_ + threshold_, compare_);
}
template <class Data,
          class CompareResult = Data,
          class Limit = Data,
          class Zero = Data,
          class Compare = MinMax<Data>>
constexpr void symmetric_threshold_result(Data& point_,
                                          CompareResult& result_,
                                          const Limit& threshold_,
                                          const Zero& zero_ = Zero(),
                                          Compare compare_ = Compare())
{
    limit(point_, result_, zero_ - threshold_, zero_ + threshold_, compare_);
}

template <class Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data,
          class Compare = MinMax<Data>>
constexpr void asymmetric_threshold(Data& point_,
                                    const LowerLimit& lower_threshold_,
                                    const UpperLimit& upper_threshold_,
                                    const Zero& zero_ = Zero(),
                                    Compare compare_ = Compare())
{
    limit(point_, zero_ - lower_threshold_, zero_ + upper_threshold_, compare_);
}
template <class Data,
          class CompareResult = Data,
          class LowerLimit = Data,
          class UpperLimit = LowerLimit,
          class Zero = Data,
          class Compare = MinMax<Data>>
constexpr void asymmetric_threshold_result(Data& point_,
                                           CompareResult& result_,
                                           const LowerLimit& lower_threshold_,
                                           const UpperLimit& upper_threshold_,
                                           const Zero& zero_ = Zero(),
                                           Compare compare_ = Compare())
{
    limit(point_,
          result_,
          zero_ - lower_threshold_,
          zero_ + upper_threshold_,
          compare_);
}
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_MATH_THRESHOLD_HPP_ */
