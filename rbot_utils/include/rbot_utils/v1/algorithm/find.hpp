#ifndef _RBOT_UTILS_V1_ALGORITHM_FIND_HPP_
#define _RBOT_UTILS_V1_ALGORITHM_FIND_HPP_

#include <algorithm>

namespace rbot_utils {
namespace v1 {
template <class InputIter, class UnaryPredicate>
#ifdef __cpp_lib_constexpr_algorithms
constexpr
#endif
InputIter find_unique(InputIter begin_,
                                InputIter end_,
                                UnaryPredicate p_) noexcept
{
    auto firstIter = std::find_if(begin_, end_, p_);
    if (firstIter == end_) {
        return end_;
    }
    auto secondIter = std::find_if(std::next(firstIter), end_, p_);
    if (secondIter != end_) {
        return end_;
    }
    return firstIter;
}
}  // namespace v1
}  // namespace rbot_utils
#endif  // _RBOT_UTILS_V1_ALGORITHM_FIND_HPP_
