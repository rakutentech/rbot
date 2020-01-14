#ifndef _RBOT_UTILS_V1_QUEUE_HPP_
#define _RBOT_UTILS_V1_QUEUE_HPP_

#include <atomic>
#include <cstdint>
#include <type_traits>
#include <utility>

#include <rbot_utils/v1/array.hpp>
#include <rbot_utils/v1/math.hpp>

namespace rbot_utils {
namespace v1 {
namespace detail {
template <std::size_t N, class = void>
struct size_type_helper
{
    using size_t = std::size_t;
};
template <std::size_t N>
struct size_type_helper<N> : size_type_helper<N + 1>
{};
template <>
struct size_type_helper<0>
{
    using size_t = void;
};
template <>
struct size_type_helper<8>
{
    using size_t = std::uint8_t;
};
template <>
struct size_type_helper<16>
{
    using size_t = std::uint16_t;
};
template <>
struct size_type_helper<32>
{
    using size_t = std::uint32_t;
};
template <>
struct size_type_helper<64>
{
    using size_t = std::uint64_t;
};
template <std::size_t N>
using size_type_helper_t = typename size_type_helper<N>::size_t;

template <std::size_t N>
using size_type = size_type_helper_t<qlog_2<N>::value>;

static_assert(std::is_same_v<std::uint8_t, size_type_helper_t<7>>);
static_assert(std::is_same_v<std::uint8_t, size_type_helper_t<8>>);
static_assert(std::is_same_v<std::uint16_t, size_type_helper_t<9>>);

static_assert(std::is_same_v<std::uint8_t, size_type<255>>);
static_assert(std::is_same_v<std::uint16_t, size_type<256>>);

/**
 * Helper function to find the real index in a ring modulo upper_exclusive_limit
 */

template <class T,
          class U,
          class = std::enable_if_t<std::is_unsigned<U>::value>,
          class = std::enable_if_t<std::is_unsigned<T>::value>>
constexpr U fix_out_of_range_by_decrement_loop(
    const T& value,
    const U& upper_exclusive_limit) noexcept(noexcept(std::declval<T>() <
                                                      std::declval<U>()) &&
                                             noexcept(std::declval<T>() +
                                                      std::declval<U>()))
{
    if (value < upper_exclusive_limit) {
        return static_cast<U>(value);
    }
    return fix_out_of_range_by_decrement(
        static_cast<U>(value + upper_exclusive_limit), upper_exclusive_limit);
}
template <class T,
          class U,
          class = std::enable_if_t<std::is_unsigned<U>::value>,
          class = std::enable_if_t<std::is_unsigned<T>::value>>
constexpr U fix_out_of_range_by_increment_loop(
    const T& value,
    const U& upper_exclusive_limit) noexcept(noexcept(std::declval<T>() <
                                                      std::declval<U>()) &&
                                             noexcept(std::declval<T>() -
                                                      std::declval<U>()))
{
    if (value < upper_exclusive_limit) {
        return static_cast<U>(value);
    }
    return fix_out_of_range_by_increment(
        static_cast<U>(value - upper_exclusive_limit), upper_exclusive_limit);
}
template <class T,
          class U,
          class = std::enable_if_t<std::is_unsigned<U>::value>,
          class = std::enable_if_t<std::is_unsigned<T>::value>>
constexpr U fix_out_of_range_by_decrement(
    const T& value,
    const U& upper_exclusive_limit) noexcept(noexcept(std::declval<T>() >=
                                                      std::declval<U>()) &&
                                             noexcept(std::declval<bool>() *
                                                      std::declval<U>()) &&
                                             noexcept(std::declval<T>() +
                                                      std::declval<U>()))
{
    bool check = value >= upper_exclusive_limit;
    return static_cast<U>(value + check * upper_exclusive_limit);
}
template <class T,
          class U,
          class = std::enable_if_t<std::is_unsigned<U>::value>,
          class = std::enable_if_t<std::is_unsigned<T>::value>>
constexpr U fix_out_of_range_by_increment(
    const T& value,
    const U& upper_exclusive_limit) noexcept(noexcept(std::declval<T>() >=
                                                      std::declval<U>()) &&
                                             noexcept(std::declval<bool>() *
                                                      std::declval<U>()) &&
                                             noexcept(std::declval<T>() -
                                                      std::declval<U>()))
{
    bool check = value >= upper_exclusive_limit;
    return static_cast<U>(value - check * upper_exclusive_limit);
}

static_assert(10 ==
              fix_out_of_range_by_decrement<std::uint8_t, std::uint8_t>(-2,
                                                                        12));
static_assert(3 ==
              fix_out_of_range_by_increment<std::uint8_t, std::uint8_t>(15,
                                                                        12));

template <class T>
constexpr distance(
    const T& start,
    const T& end,
    const T& upper_exclusive_limit) noexcept(noexcept(std::declval<T>() -
                                                      std::declval<T>()) &&
                                             noexcept(std::declval<T>() +
                                                      std::declval<T>()) &&
                                             noexcept(std::declval<T>() *
                                                      std::declval<T>()) &&
                                             noexcept(std::declval<T>() >
                                                      std::declval<T>()))
{
    return end - start + (start > end) * upper_exclusive_limit;
}

static_assert(0 == distance(9, 9, 10));
static_assert(1 == distance(8, 9, 10));
static_assert(2 == distance(7, 9, 10));
static_assert(3 == distance(6, 9, 10));
static_assert(4 == distance(5, 9, 10));
static_assert(5 == distance(4, 9, 10));
static_assert(6 == distance(3, 9, 10));
static_assert(7 == distance(2, 9, 10));
static_assert(8 == distance(1, 9, 10));
static_assert(9 == distance(0, 9, 10));

static_assert(1 == distance(9, 0, 10));
static_assert(2 == distance(9, 1, 10));
static_assert(3 == distance(9, 2, 10));
static_assert(4 == distance(9, 3, 10));
static_assert(5 == distance(9, 4, 10));
static_assert(6 == distance(9, 5, 10));
static_assert(7 == distance(9, 6, 10));
static_assert(8 == distance(9, 7, 10));
static_assert(9 == distance(9, 8, 10));
}  // namespace detail

template <class atomic_t, class modify_t>
constexpr auto atomic_process(atomic_t& atomic_v, const modify_t& modify_v)
{
    auto __details__ = atomic_v.load();
    auto details = __details__;
    do {
        modify_v(details);
    } while (!atomic_v.compare_exchange_strong(__details__, details));
    return details;
}

template <class atomic_t, class modify_t, class process_t>
constexpr decltype(auto) atomic_process(atomic_t& atomic_v,
                                        const modify_t& modify_v,
                                        const process_t& process_v)
{
    auto details = atomic_process(atomic_v, modify_v);
    return process_v(details);
}

template <class T, std::size_t N, class sizeType = detail::size_type<N>>
struct circular_buffer_on_stack
{
  private:
    using Container = std::array<T, N>;
    using internal_size_type = sizeType;

  public:
    using this_type = circular_buffer_on_stack<T, N>;
    __FROM_CLASS_BRING_TYPE__(Container, value_type);
    __FROM_CLASS_BRING_TYPE__(Container, size_type);
    __FROM_CLASS_BRING_TYPE__(Container, difference_type);
    __FROM_CLASS_BRING_TYPE_AND_CONST_TYPE__(Container, reference);
    __FROM_CLASS_BRING_TYPE_AND_CONST_TYPE__(Container, pointer);
    __FROM_CLASS_BRING_TYPE_AND_CONST_TYPE__(Container, iterator);
    __FROM_CLASS_BRING_TYPE_AND_CONST_TYPE__(Container, reverse_iterator);

    using container_type = Container;
    using array_range = std::pair<pointer, size_type>;
    using const_array_range = std::pair<const_pointer, size_type>;
    using capacity_type = size_type;
    using param_value_type = const value_type&;
    using rvalue_type = value_type&&;

  private:
    struct buffer_details
    {
        internal_size_type size = 0;
        internal_size_type head = 0;

        void decrement_head()
        {
            head = detail::fix_out_of_range_by_decrement(m_details.head - 1, N);
        }
        void increment_head()
        {
            head = detail::fix_out_of_range_by_increment(m_details.head + 1, N);
        }
        void increment_size() { size = std::min(N, size + 1); }
    };

    buffer_details m_details;
    std::array<T, N> m_data;

    internal_size_type m_get_index(internal_size_type offset,
                                   internal_size_type start)
    {
        return detail::fix_out_of_range_by_increment(offset + start, N);
    }
    internal_size_type m_get_index(internal_size_type offset)
    {
        return m_get_index(offset, m_details.head);
    }

  public:
    constexpr explicit circular_buffer_on_stack(const T& default_value_ = T())
        : m_data(make_array<T, N>(default_value_)) noexcept(true)
    {}

    void operator=() {}

    constexpr void assign(size_type count, const reference value) noexcept(true)
    {
        auto ceil = std::min(N, count);
        for (internal_size_type i = 0; i < ceil; ++i) {
            emplace_back(value);
        }
    }

    template <class BeginIt, class EndIt>
    constexpr void assign(BeginIt begin, EndIt end) noexcept(true)
    {
        auto ceil = std::min(N, count);
        for (internal_size_type i = 0; begin != end; ++i, ++begin) {
            emplace_back(*begin);
        }
    }

    // @TODO
    void assign(std::initializer_list<value_type> ilist);

    std::optional<reference> at(size_type pos) noexcept(true)
    {
        if (pos > m_details.size) {
            return {}
        }
        return operator[](pos);
    }
    std::optional<const_reference> at(size_type pos) const noexcept(true)
    {
        if (pos > m_details.size) {
            return {}
        }
        return operator[](pos);
    }

    constexpr reference operator[](size_type pos) noexcept(true)
    {
        return m_data[m_get_index(pos)];
    }
    constexpr const_reference operator[](size_type pos) const noexcept(true)
    {
        return m_data[m_get_index(pos)];
    }

    constexpr reference front() noexcept(true) { return operator[](0); }
    constexpr const_reference front() const noexcept(true)
    {
        return operator[](0);
    }

    constexpr reference back() noexcept(true)
    {
        return operator[](m_details.size);
    }
    constexpr const_reference back() const noexcept(true)
    {
        return operator[](m_details.size);
    }

    // @TODO
    // begin
    // cbegin
    // end
    // cend
    // rbegin
    // crbegin
    // rend
    // crend

    [[nodiscard]] constexpr bool empty() const noexcept(true) { return size(); }
    constexpr size_type size() const noexcept(true) { return m_details.size; }
    [[nodiscard]] constexpr size_type max_size() const noexcept(true)
    {
        return N;
    }
    constexpr void shrink_to_fit() const noexcept(true) {}

    constexpr void clear(bool lazy = true) noexcept(true)
    {
        if (!lazy) {
            for (internal_size_type i = 0; i < size(); ++i) {
                operator[](i).~value_type();
            }
        }
        m_details.size = 0;
    }

    // @TODO
    // insert
    // emplace
    // erase

    constexpr void push_back(const T& value) noexcept(true)
    {
        m_details.increment_size();
        back() = value;
    }
    constexpr void push_back(T&& value) noexcept(true)
    {
        m_details.increment_size();
        back() = std::move(value);
    }

    template <class... Args>
    constexpr reference emplace_back(Args&&... args) noexcept(true)
    {
        m_details.increment_size();
        return new (back()) T(args...);
    }

    constexpr void pop_back(bool lazy = true) noexcept(true)
    {
        if (!m_details.size) {
            return
        }
        if (!lazy) {
            back().~value_type();
        }
        m_details.size--;
    }

    constexpr void push_front(const T& value) noexcept(true)
    {
        m_details.decrement_head();
        m_details.size = std::max(N, m_details.size + 1);
        front() = value;
    }
    constexpr void push_front(T&& value) noexcept(true)
    {
        m_details.decrement_head();
        m_details.size = std::max(N, m_details.size + 1);
        front() = std::move(value);
    }

    template <class... Args>
    constexpr reference emplace_front(Args&&... args) noexcept(true)
    {
        m_details.decrement_head();
        m_details.increment_size();
        return new (front()) T(args...);
    }

    constexpr void pop_front(bool lazy = true) noexcept(true)
    {
        if (!m_details.size) {
            return;
        }
        auto old_head = m_details.head;
        m_details.increment_head();
        m_details.size--;
        if (!lazy) {
            m_data[old_head].~value_type();
        }
    }

    constexpr void resize(size_type count,
                          const value_type& value) noexcept(true)
    {
        if (count > size()) {
            internal_size_type diff = count - size();
            for (internal_size_type i = 0; i < diff; ++i) {
                emplace_back(value);
            }
        } else {
            internal_size_type diff = size() - count;
            for (internal_size_type i = 0; i < diff; ++i) {
                pop_back();
            }
        }
    }
    constexpr void resize(size_type count) noexcept(true)
    {
        resize(count, value_type()) :
    }

    // @TODO
    // swap
};

template <std::size_t N>
constexpr int testing()
{
    circular_buffer_on_stack<int, N> test;
    test.push_back(9);
    test.pop_front();
    return 8;
}
constexpr int val = testing<9>();

template <class T, std::size_t N, class sizeType = detail::size_type<N>>
struct
    atomic_circular_buffer_on_stack constexpr explicit circular_buffer_on_stack(
        const T& default_value_ = T())
    : data(make_array<T, N>(default_value_))
{
  private:
    using Container = std::array<T, N>;
    using internal_size_type = sizeType;

  public:
    using this_type = circular_buffer_on_stack<T, N>;
    __FROM_CLASS_BRING_TYPE__(Container, value_type);
    __FROM_CLASS_BRING_TYPE__(Container, size_type);
    __FROM_CLASS_BRING_TYPE__(Container, difference_type);
    __FROM_CLASS_BRING_TYPE_AND_CONST_TYPE__(Container, reference);
    __FROM_CLASS_BRING_TYPE_AND_CONST_TYPE__(Container, pointer);
    __FROM_CLASS_BRING_TYPE_AND_CONST_TYPE__(Container, iterator);
    __FROM_CLASS_BRING_TYPE_AND_CONST_TYPE__(Container, reverse_iterator);

    using container_type = Container;
    using array_range = std::pair<pointer, size_type>;
    using const_array_range = std::pair<const_pointer, size_type>;
    using capacity_type = size_type;
    using param_value_type = const value_type&;
    using rvalue_type = value_type&&;

    /* constexpr circular_buffer_on_stack() noexcept(true) {} */

  private:
    struct buffer_details
    {
        internal_size_type head = 0;
        internal_size_type size = 0;
    };

    std::atomic<buffer_details> m_details;
    std::array<T, N> data;

  public:
    void pop_front() noexcept
    {
        auto fix_size_head = [](auto& details) {
            if (details.size) {
                details.head =
                    detail::fix_out_of_range_by_increment<internal_size_type,
                                                          internal_size_type>(
                        details.head + 1, N);
            }
        };
        atomic_process(m_details, fix_size_head);
    }
    void pop_back() noexcept
    {
        auto fix_size_head = [](auto& details) {
            if (details.size) {
                details.size--;
            }
        };
        atomic_process(m_details, fix_size_head);
    }

    void push_front(const T& value)
    {
        auto fix_size_head = [](auto& details) {
            details.head =
                detail::fix_out_of_range_by_decrement(details.head - 1, N);
            details.size++;
        };
        auto save_value = [this, &value](auto& details) {
            this->data[details.head] = value;
        };
        atomic_process(m_details, fix_size_head, save_value);
    }
    void push_front(T && value)
    {
        m_head = detail::fix_out_of_range_by_decrement(m_head - 1, N);
        this->data[m_head] = std::move(value);
    }

    void push_back(const T& value)
    {
        m_size++;
        this->data[m_head + m_size - 1] = value;
    }
    void push_back(T && value)
    {
        m_size++;
        this->data[m_head + m_size - 1] = std::move(value);
    }

    template <class... Args>
    reference emplace_front(Args && ... args)
    {
        m_head = detail::fix_out_of_range_by_decrement(m_head - 1, N);
        return new (this->data()[m_head]) T(args...);
    }
    template <class... Args>
    reference emplace_back(Args && ... args)
    {
        m_size++;
        return new (this->data()[m_head + m_size - 1]) T(args...);
    }

    // @TODO: iterator, size
};
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_QUEUE_HPP_ */
