#ifndef _RBOT_UTILS_V1_MATH_LOG_HPP_
#define _RBOT_UTILS_V1_MATH_LOG_HPP_

namespace rbot_utils {
namespace v1 {
template <std::size_t N, class = void>
struct qlog_2
{
    static constexpr std::size_t value = 0;
};

template <>
struct qlog_2<1>
{
    static constexpr std::size_t value = 1;
};

template <>
struct qlog_2<2>
{
    static constexpr std::size_t value = 2;
};

template <std::size_t N>
struct qlog_2<N>
{
    static constexpr std::size_t value = qlog_2<(N >> 1)>::value + 1;
};

static_assert(3 == qlog_2<7>::value);
static_assert(4 == qlog_2<8>::value);
static_assert(4 == qlog_2<9>::value);
}  // namespace v1
}  // namespace rbot_utils
#endif /* _RBOT_UTILS_V1_MATH_LOG_HPP_ */
