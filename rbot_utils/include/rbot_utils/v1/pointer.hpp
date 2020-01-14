#ifndef _RBOT_UTILS_V1_POINTER_HPP_
#define _RBOT_UTILS_V1_POINTER_HPP_

#include <memory>
#include <type_traits>
#include <utility>

namespace rbot_utils {
namespace v1 {
template <class To, class From>
std::unique_ptr<To> static_pointer_cast(std::unique_ptr<From> &&from)
{
    return std::unique_ptr<To>(static_cast<To *>(from.release()));
}

namespace detail {
template <class T>
struct is_unique_ptr : public std::false_type
{};
template <class Type, class Alloc>
struct is_unique_ptr<std::unique_ptr<Type, Alloc>> : public std::true_type
{};
template <class T, class = void>
struct is_shared_ptr : public std::false_type
{};
template <class Type>
struct is_shared_ptr<std::shared_ptr<Type>> : public std::true_type
{};
}  // namespace detail

using unique_void_ptr = std::unique_ptr<void, void (*)(void const *)>;

template <class T>
auto unique_void(T *ptr) -> unique_void_ptr
{
    return unique_void_ptr(ptr, [](void const *data) {
        T const *ptr = static_cast<T const *>(data);
        delete ptr;
    });
}

template <class T, class... Args>
auto make_unique_void(Args &&... args) -> unique_void_ptr
{
    return unique_void(new T(std::forward<Args>(args)...));
}

template <class T>
auto make_unique_void_default_init() -> unique_void_ptr
{
    return unique_void(new T());
}

template <class T, class... Args>
decltype(auto) smartFactory(Args &&... args)
{
    if constexpr (detail::is_unique_ptr<T>::value) {
        return std::make_unique<typename T::element_type>(
            std::forward<Args>(args)...);
    } else if constexpr (detail::is_shared_ptr<T>::value) {
        return std::make_shared<typename T::element_type>(
            std::forward<Args>(args)...);
    } else if constexpr (std::is_pointer<T>::value) {
        return new std::remove_pointer_t<T>(std::forward<Args>(args)...);
    } else {
        return T(std::forward<Args>(args)...);
    }
}

namespace detail {
template <class T, class = void>
struct is_smart_pointer : std::false_type
{};
template <class T>
struct is_smart_pointer<
    T,
    // check if T::element_type* get() is defined
    std::enable_if_t<std::is_same<std::add_pointer_t<typename T::element_type>,
                                  decltype(std::declval<T>().get())>::value>>
    : std::true_type
{};
}  // namespace detail
template <class T>
struct is_smart_pointer : detail::is_smart_pointer<std::remove_cv_t<T>>
{};

static_assert(is_smart_pointer<std::shared_ptr<int>>::value);
static_assert(is_smart_pointer<std::unique_ptr<int>>::value);
static_assert(!is_smart_pointer<int>::value);
static_assert(!is_smart_pointer<int *>::value);
static_assert(!is_smart_pointer<int &>::value);
}  // namespace v1
}  // namespace rbot_utils

#ifdef BOOST_VERSION
#include <boost/smart_ptr.hpp>
namespace rbot_utils {
template <class SharedPointer>
struct Holder
{
    SharedPointer p;

    Holder(const SharedPointer &p) : p(p) {}
    Holder(const Holder &other) : p(other.p) {}
    Holder(Holder &&other) : p(std::move(other.p)) {}

    void operator()(...) { p.reset(); }
};

namespace v1 {
template <class T>
std::shared_ptr<T> to_std_shared_ptr(const boost::shared_ptr<T> &p)
{
    using Ptr = Holder<std::shared_ptr<T>>;
    if (Ptr *h = boost::get_deleter<Ptr>(p)) {
        return h->p;
    } else {
        return std::shared_ptr<T>(p.get(), Holder<boost::shared_ptr<T>>(p));
    }
}

template <class T>
boost::shared_ptr<T> to_boost_shared_ptr(const std::shared_ptr<T> &p)
{
    using Ptr = Holder<boost::shared_ptr<T>>;
    if (Ptr *h = std::get_deleter<Ptr>(p)) {
        return h->p;
    } else {
        return boost::shared_ptr<T>(p.get(), Holder<std::shared_ptr<T>>(p));
    }
}
static_assert(rbot_utils::is_smart_pointer<boost::shared_ptr<int>>::value);
static_assert(rbot_utils::is_smart_pointer<boost::scoped_ptr<int>>::value);
}  // namespace v1
}  // namespace rbot_utils
#endif
#endif /* ifndef _RBOT_UTILS_V1_POINTER_HPP_ */
