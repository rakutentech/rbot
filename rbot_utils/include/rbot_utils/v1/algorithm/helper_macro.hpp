#ifndef _RBOT_UTILS_V1_ALGORITHM_HELPER_MACRO_HPP_
#define _RBOT_UTILS_V1_ALGORITHM_HELPER_MACRO_HPP_

namespace rbot_utils {
namespace v1 {
namespace internal {
#define __FROM_LOWER_UPPER__(LOWER, UPPER)                         \
    template <class NameCase, class Method, class T>               \
    struct from_##LOWER                                            \
    {};                                                            \
    template <class T>                                             \
    struct from_##LOWER<lower, data_member, T>                     \
    {                                                              \
        constexpr const decltype(auto) operator()(const T &object) \
        {                                                          \
            return object.LOWER;                                   \
        }                                                          \
    };                                                             \
    template <class T>                                             \
    struct from_##LOWER<upper, data_member, T>                     \
    {                                                              \
        constexpr const decltype(auto) operator()(const T &object) \
        {                                                          \
            return object.UPPER;                                   \
        }                                                          \
    };                                                             \
    template <class T>                                             \
    struct from_##LOWER<lower, getter, T>                          \
    {                                                              \
        constexpr decltype(auto) operator()(const T &object)       \
        {                                                          \
            return object.LOWER();                                 \
        }                                                          \
    };                                                             \
    template <class T>                                             \
    struct from_##LOWER<upper, getter, T>                          \
    {                                                              \
        constexpr decltype(auto) operator()(const T &object)       \
        {                                                          \
            return object.UPPER();                                 \
        }                                                          \
    }

#define __TO_LOWER_UPPER__(LOWER, UPPER)                      \
    template <class NameCase, class Method, class T, class U> \
    struct to_##LOWER                                         \
    {};                                                       \
    template <class T, class U>                               \
    struct to_##LOWER<lower, data_member, T, U>               \
    {                                                         \
        /*                                                    \
        constexpr T &operator()(T &object, const U &&value)   \
        {                                                     \
            object.LOWER = value;                             \
            return object;                                    \
        }                                                     \
        */                                                    \
        constexpr T &operator()(T &object, const U &value)    \
        {                                                     \
            object.LOWER = value;                             \
            return object;                                    \
        }                                                     \
    };                                                        \
    template <class T, class U>                               \
    struct to_##LOWER<upper, data_member, T, U>               \
    {                                                         \
        constexpr T &operator()(T &object, const U &value)    \
        {                                                     \
            object.UPPER = value;                             \
            return object;                                    \
        }                                                     \
    };                                                        \
    template <class T, class U>                               \
    struct to_##LOWER<lower, getter, T, U>                    \
    {                                                         \
        constexpr T &operator()(T &object, const U &value)    \
        {                                                     \
            object.LOWER() = value;                           \
            return object;                                    \
        }                                                     \
    };                                                        \
    template <class T, class U>                               \
    struct to_##LOWER<upper, getter, T, U>                    \
    {                                                         \
        constexpr T &operator()(T &object, const U &value)    \
        {                                                     \
            object.UPPER() = value;                           \
            return object;                                    \
        }                                                     \
    };                                                        \
    template <class T, class U>                               \
    struct to_##LOWER<lower, setter, T, U>                    \
    {                                                         \
        constexpr T &operator()(T &object, const U &value)    \
        {                                                     \
            object.LOWER(value);                              \
            return object;                                    \
        }                                                     \
    };                                                        \
    template <class T, class U>                               \
    struct to_##LOWER<upper, setter, T, U>                    \
    {                                                         \
        constexpr T &operator()(T &object, const U &value)    \
        {                                                     \
            object.UPPER(value);                              \
            return object;                                    \
        }                                                     \
    }
}  // namespace internal

#define __FROM_INTERNAL__(LOWER)                                      \
    template <class T>                                                \
    constexpr decltype(auto) from_##LOWER(const T &object)            \
    {                                                                 \
        internal::from_##LOWER<typename LOWER##_traits<T>::name_case, \
                               typename LOWER##_traits<T>::method,    \
                               T>                                     \
            fn;                                                       \
        return fn(object);                                            \
    }

/* @TODO: use auto in fn parameter with concepts later */
#define __TO_INTERNAL__(LOWER)                                      \
    template <class T, class U>                                     \
    constexpr T &to_##LOWER(T &object, const U &value)              \
    {                                                               \
        internal::to_##LOWER<typename LOWER##_traits<T>::name_case, \
                             typename LOWER##_traits<T>::method,    \
                             T,                                     \
                             decltype(value)>                       \
            fn;                                                     \
        return fn(object, value);                                   \
    }                                                               \
    template <class T, class U>                                     \
    constexpr T &to_##LOWER(T &object, const U &&value)             \
    {                                                               \
        internal::to_##LOWER<typename LOWER##_traits<T>::name_case, \
                             typename LOWER##_traits<T>::method,    \
                             T,                                     \
                             decltype(value)>                       \
            fn;                                                     \
        return fn(object, value);                                   \
    }
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_ALGORITHM_HELPER_MACRO_HPP_ */
