#ifndef _RBOT_UTILS_MATH_FILTER_HPP_
#define _RBOT_UTILS_MATH_FILTER_HPP_

#include <type_traits>
#include <utility>

namespace rbot_utils {
namespace v1 {
template <class T>
struct Filter
{};

template <class data_t, class parameter_t = double>
struct ExponentialFilter : Filter<data_t>
{
  protected:
    template <class Data, class Parameter, class = void>
    struct internal_data
    {
        Data data;
        Parameter k;
        internal_data(Data data_, Parameter k_) : data(data_), k(k_) {}
    };
    template <class Data, class Parameter>
    struct internal_data<Data,
                         Parameter,
                         std::enable_if_t<(sizeof(Data) < sizeof(Parameter))>>
    {
        Parameter k;
        Data data;
        internal_data(Data data_, Parameter k_) : k(k_), data(data_) {}
    };

    internal_data<data_t, parameter_t> m_internal;

  public:
    constexpr ExponentialFilter(parameter_t k,
                                data_t initial_value = data_t(0)) noexcept(true)
        : m_internal(initial_value, k)
    {}

    constexpr void reset(data_t data) noexcept(true) { m_internal.data = data; }

    constexpr data_t &filter(data_t new_data) noexcept(true)
    {
        m_internal.data =
            m_internal.k * (new_data - m_internal.data) + m_internal.data;
        return m_internal.data;
    }

    constexpr void set_factor(parameter_t k) noexcept(true)
    {
        m_internal.k = k;
    }

    constexpr data_t &data() noexcept(true) { return m_internal.data; }
};

template <class data_t, std::size_t window_size = 5>
struct ConstantWindowAvgFilter : Filter<data_t>
{
  protected:
    std::array<data_t, window_size> m_history;
    data_t m_data;

  public:
    constexpr ConstantWindowAvgFilter(/*@ TODO*/) {}

    constexpr void reset(data_t data) noexcept(true)
    {
        m_history.clear();
        m_data = data;
    }

    constexpr data_t &filter(data_t new_data) noexcept(true)
    { /* @TODO */
    }

    constexpr data_t &data() noexcept(true) { return m_data; }
};
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_MATH_FILTER_HPP_ */
