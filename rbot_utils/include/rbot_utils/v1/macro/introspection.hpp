#ifndef _RBOT_UTILS_V1_MACRO_INTROSPECTION_HPP_
#define _RBOT_UTILS_V1_MACRO_INTROSPECTION_HPP_

#include <type_traits>
#include <utility>

namespace rbot_utils {
namespace v1 {
namespace detail {
#ifdef __cpp_lib_void_t
template <class... Args>
using void_t = std::void_t<Args...>;
#else
template <class...>
using void_t = void;
#endif
}  // namespace detail

#define ADD_HELPER_VALUE_TEMPLATE(Name, Query) \
    template <class Query>                     \
    inline constexpr auto Name##_v = Name<Query>::value

#define ADD_HELPER_VALUE_SFINAE(Name, Variable, Query) \
    template <class Query>                             \
    using Name = std::enable_if_t<Variable<Query>, bool>

#define ADD_HELPER_TYPE_TEMPLATE(Name, Query) \
    template <class Query>                    \
    using Name##_t = typename Name<Query>::type

#define SETUP_HAS_PUBLIC_DATA_MEMBER(Name)                             \
    template <class Query, class = void>                               \
    struct has_public_data_member_##Name : public std::false_type      \
    {};                                                                \
    template <class Query>                                             \
    struct has_public_data_member_##Name<                              \
        Query,                                                         \
        detail::void_t<decltype(Query::Name)>> : public std::true_type \
    {};                                                                \
    ADD_HELPER_VALUE_TEMPLATE(has_public_data_member_##Name, Query);   \
    ADD_HELPER_VALUE_SFINAE(                                           \
        HasPublicDataMember_##Name, has_public_data_member_##Name##_v, Query)

#define SETUP_HAS_PUBLIC_FUNCTOR(Name)                           \
    template <class Query, class = void>                         \
    struct has_public_functor_##Name : public std::false_type    \
    {};                                                          \
    template <class Query>                                       \
    struct has_public_functor_##Name<                            \
        Query,                                                   \
        detail::void_t<decltype(std::declval<Query>().Name())>>  \
        : public std::true_type                                  \
    {};                                                          \
    ADD_HELPER_VALUE_TEMPLATE(has_public_functor_##Name, Query); \
    ADD_HELPER_VALUE_SFINAE(                                     \
        HasPublicFunctor_##Name, has_public_functor_##Name##_v, Query)

#define SETUP_HAS_PUBLIC_TYPE_ALIAS(Name)                                     \
    template <class Query, class = void>                                      \
    struct has_public_type_alias_##Name : public std::false_type              \
    {};                                                                       \
    template <class Query>                                                    \
    struct has_public_type_alias_##Name<Query,                                \
                                        detail::void_t<typename Query::Name>> \
        : public std::true_type                                               \
    {};                                                                       \
    ADD_HELPER_VALUE_TEMPLATE(has_public_type_alias_##Name, Query);           \
    ADD_HELPER_VALUE_SFINAE(                                                  \
        HasPublicTypeAlias_##Name, has_public_type_alias_##Name##_v, Query)
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_MACRO_INTROSPECTION_HPP_ */
