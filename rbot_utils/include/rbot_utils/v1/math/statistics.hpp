#ifndef _RBOT_UTILS_V1_MATH_STATISTICS_HPP_
#define _RBOT_UTILS_V1_MATH_STATISTICS_HPP_

#include <type_traits>

#include <rbot_utils/pointer.hpp>

namespace rbot_utils {
namespace v1 {
template <class T>
struct is_compute_base
    : std::conjunction<std::negation<rbot_utils::is_smart_pointer<T>>,
                       std::negation<std::is_pointer<T>>>
{};

// @TODO: allow variance to reduce double computation of average
// * return average, variance
// * take in average to reduce operation
#define __MATH_BASIC_OPERATION__(NAME)                                     \
    template <class ValueType,                                             \
              class InputType,                                             \
              class = std::enable_if_t<is_compute_base<InputType>::value>, \
              class = void>                                                \
    constexpr ValueType NAME(const InputType &input) noexcept(true);       \
                                                                           \
    template <class ValueType, class InputType>                            \
    constexpr ValueType NAME(const InputType *const input) noexcept        \
    {                                                                      \
        {                                                                  \
            if (input == nullptr) {                                        \
                return ValueType();                                        \
            }                                                              \
            return NAME<ValueType, InputType>(*input);                     \
        }                                                                  \
    }                                                                      \
                                                                           \
    template <class ValueType,                                             \
              class SmartPtr,                                              \
              class = std::enable_if_t<                                    \
                  rbot_utils::is_smart_pointer<SmartPtr>::value>>          \
    constexpr ValueType NAME(SmartPtr ptr) noexcept(true)                  \
    {                                                                      \
        return NAME<ValueType>(ptr.get());                                 \
    }

__MATH_BASIC_OPERATION__(average);
__MATH_BASIC_OPERATION__(variance);

// @TODO: take in operations for number 1, +, -, * and /
template <class InputType,
          class SizeType,
          class InputPlusInput = std::plus<InputType>,
          class InputMinusInput = std::minus<InputType>>
constexpr InputType running_average(
    const InputType &input,
    const InputType &current_average,
    const SizeType &current_size,
    InputPlusInput plus = InputPlusInput(),
    InputMinusInput minus = InputMinusInput()) noexcept(true)
{
    /* current_average + (input - current_average) / (current_size + 1); */
    return plus(current_average,
                minus(input, current_average) / (current_size + 1));
}

template <class InputType,
          class SizeType,
          class InputPlusInput = std::plus<InputType>,
          class InputMinusInput = std::minus<InputType>>
constexpr InputType modified_average(
    const InputType &current_input,
    const InputType &new_input,
    const InputType &current_average,
    const SizeType &current_size,
    InputPlusInput plus = InputPlusInput(),
    InputMinusInput minus = InputMinusInput()) noexcept(true)
{
    /* current_average + (new_input - current_input) / current_size; */
    return plus(current_average,
                (minus(new_input, current_input) / current_size));
}

template <class InputType,
          class SizeType,
          class InputMultipliesInput = std::multiplies<InputType>,
          class InputPlusInput = std::plus<InputType>,
          class InputMinusInput = std::minus<InputType>>
constexpr InputType running_variance(
    const InputType &input,
    const InputType &current_average,
    const InputType &new_average,
    const InputType &current_variance,
    const SizeType &current_size,
    bool population = false,
    bool m2_not_variance = false,  // variance = m2/(current_size + population)
    InputMultipliesInput multiplies = InputMultipliesInput(),
    InputPlusInput plus = InputPlusInput(),
    InputMinusInput minus = InputMinusInput()) noexcept(true)
{
    if (current_size < 2) {
        return current_variance;
    }
    auto divisor = (current_size - 1 + population);

    InputType m2, old_m2;

    if (m2_not_variance) {
        old_m2 = current_variance;
    } else {
        old_m2 = current_variance * divisor;
    }

    m2 = plus(
        old_m2,
        multiplies(minus(input, current_average), minus(input, new_average)));

    if (m2_not_variance) {
        return m2;
    }
    // size has now increased by 1
    return m2 / (divisor + 1);
}

template <class InputType,
          class SizeType,
          class InputMultipliesInput = std::multiplies<InputType>,
          class InputPlusInput = std::plus<InputType>,
          class InputMinusInput = std::minus<InputType>>
constexpr InputType modified_variance(
    const InputType &current_input,
    const InputType &new_input,
    const InputType &current_average,
    const InputType &new_average,
    const InputType &current_variance,
    const SizeType &current_size,
    bool population = false,
    InputMultipliesInput multiplies = InputMultipliesInput(),
    InputPlusInput plus = InputPlusInput(),
    InputMinusInput minus = InputMinusInput()) noexcept(true)
{
    if (current_size < 2) {
        return current_variance;
    }
    auto divisor = (current_size - 1 + population);
    auto delta = minus(new_average, current_average);
    return plus(current_variance,
                multiplies(delta,
                           plus(minus(new_input, new_average),
                                minus(current_input, current_average)) /
                               divisor));
}

template <class BufferType,
          class ValueType = typename BufferType::value_type,
          class VarianceType = ValueType,
          class Multiplication = std::multiplies<ValueType>,
          class Addition = std::plus<ValueType>,
          class Subtraction = std::minus<ValueType>>
struct Statistics
{
    BufferType buffer;
    ValueType mean;
    VarianceType variance;

    template <class NewValue>
    void compute(NewValue new_value)
    {
        ValueType new_mean;
        VarianceType new_var;

        if (buffer.size() == buffer.capacity()) {
            ValueType old_value = buffer.front();
            new_mean = rbot_utils::modified_average<
                ValueType,
                decltype(std::declval<BufferType>().size()),
                Addition,
                Subtraction>(old_value, new_value, mean, buffer.size());
            new_var = rbot_utils::modified_variance<
                ValueType,
                decltype(std::declval<BufferType>().size()),
                Multiplication,
                Addition,
                Subtraction>(
                old_value, new_value, mean, new_mean, variance, buffer.size());
        } else {
            new_mean = rbot_utils::running_average<
                ValueType,
                decltype(std::declval<BufferType>().size()),
                Addition,
                Subtraction>(new_value, mean, buffer.size());
            new_var = rbot_utils::running_variance<
                ValueType,
                decltype(std::declval<BufferType>().size()),
                Multiplication,
                Addition,
                Subtraction>(
                new_value, mean, new_mean, variance, buffer.size());
        }

        // @TODO: push_back vs emplace_back
        buffer.push_back(new_value);
        mean = new_mean;
        variance = new_var;
    }
};
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_MATH_STATISTICS_HPP_ */
