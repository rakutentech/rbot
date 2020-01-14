#ifndef _RBOT_UTILS_V1_ARRAY_H_
#define _RBOT_UTILS_V1_ARRAY_H_

#include <array>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

namespace rbot_utils {
namespace v1 {
namespace detail {
// This allows faking std::array<T, N>::fill(T val) for T with no default ctor
template <class T, std::size_t... N>
constexpr std::array<T, sizeof...(N)> repeat(
    const T &value_, std::index_sequence<N...>) noexcept(true)
{
    // unpack N, repeating value_ sizeof...(N) times
    // (N, value_) returns value
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-value"
    return {{(N, value_)...}};
#pragma GCC diagnostic pop
}

template <class T, std::size_t N>
constexpr std::array<T, N> make_array(const T &value_ = T()) noexcept(true)
{
    // std::make_index_sequence<N> is 0, 1, 2, ... N-1
    return repeat(value_, std::make_index_sequence<N>());
}
template <class T, std::size_t N>
[[deprecated("Use make_array instead")]] constexpr std::array<T, N>
create_array(const T &value_ = T()) noexcept(true)
{
    return make_array<T, N>(value_);
}

#if __cplusplus < 201703L
template <class T,
          std::size_t MasterCopySize,
          std::size_t NewSize,
          std::size_t Offset>
constexpr auto fill_array_offset(
    const std::array<T, MasterCopySize> &master_copy_array_, const T &value_)
    -> std::enable_if_t<(MasterCopySize < NewSize), std::array<T, NewSize>>
{
    static_assert(NewSize >= MasterCopySize + Offset,
                  "Insufficient size for the given offset. Reduce offset");
    auto fullArray = make_array<T, NewSize>(value_);
    for (std::size_t i = 0; i < master_copy_array_.size(); ++i) {
        fullArray[i + Offset] = master_copy_array_[i];
    }
    return fullArray;
}
template <class T,
          std::size_t MasterCopySize,
          std::size_t NewSize,
          std::size_t Offset>
constexpr auto fill_array_offset(
    const std::array<T, MasterCopySize> &master_copy_array_, const T &value_)
    -> std::enable_if_t<(MasterCopySize == NewSize), std::array<T, NewSize>>
{
    static_assert(NewSize >= MasterCopySize + Offset,
                  "Insufficient size for the given offset. Reduce offset");
    return master_copy_array_;
}
#else
template <class T,
          std::size_t MasterCopySize,
          std::size_t NewSize,
          std::size_t Offset>
constexpr auto fill_array_offset(
    const std::array<T, MasterCopySize> &master_copy_array_, const T &value_)
    -> std::array<T, NewSize>
{
    static_assert(NewSize >= MasterCopySize,
                  "Can't create array by dropping elements");
    static_assert(NewSize >= MasterCopySize + Offset,
                  "Insufficient size for the given offset. Reduce offset");
    if constexpr (NewSize > MasterCopySize) {
        auto fullArray = make_array<T, NewSize>(value_);
        for (std::size_t i = 0; i < master_copy_array_.size(); ++i) {
            fullArray[i + Offset] = master_copy_array_[i];
        }
        return fullArray;
    } else if constexpr (NewSize == MasterCopySize) {
        return master_copy_array_;
    }
}
#endif
}  // namespace detail

using detail::create_array;
using detail::make_array;

template <class T, std::size_t MasterCopySize, std::size_t NewSize>
constexpr auto post_fix_array(
    const std::array<T, MasterCopySize> &master_copy_array_, const T &value_)
    -> std::array<T, NewSize>
{
    return detail::fill_array_offset<T, MasterCopySize, NewSize, 0>(
        master_copy_array_, value_);
}

template <class T, std::size_t MasterCopySize, std::size_t NewSize>
constexpr auto pre_fix_array(
    const std::array<T, MasterCopySize> &master_copy_array_, const T &value_)
    -> std::array<T, NewSize>
{
    return detail::fill_array_offset<T,
                                     MasterCopySize,
                                     NewSize,
                                     (NewSize - MasterCopySize)>(
        master_copy_array_, value_);
}

template <class Container, std::size_t N>
using array_of_container = std::array<Container, N>;

template <class T, std::size_t N, class Allocator = std::allocator<T>>
using array_of_vector = array_of_container<std::vector<T, Allocator>, N>;
template <class T, std::size_t N, std::size_t M>
using array_2d = array_of_container<std::array<T, M>, N>;
}  // namespace v1
}  // namespace rbot_utils
#endif  // _RBOT_UTILS_V1_ARRAY_H_
