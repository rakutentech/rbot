#ifndef _RBOT_UTILS_V1_TYPE_TRAITS_HPP_
#define _RBOT_UTILS_V1_TYPE_TRAITS_HPP_

#include <type_traits>
#include <utility>

#include <rbot_utils/v1/macro/introspection.hpp>

namespace rbot_utils {
namespace v1 {
namespace detail {
SETUP_HAS_PUBLIC_DATA_MEMBER(x);
SETUP_HAS_PUBLIC_DATA_MEMBER(y);
SETUP_HAS_PUBLIC_DATA_MEMBER(z);
SETUP_HAS_PUBLIC_DATA_MEMBER(w);

SETUP_HAS_PUBLIC_DATA_MEMBER(X);
SETUP_HAS_PUBLIC_DATA_MEMBER(Y);
SETUP_HAS_PUBLIC_DATA_MEMBER(Z);
SETUP_HAS_PUBLIC_DATA_MEMBER(W);

SETUP_HAS_PUBLIC_FUNCTOR(x);
SETUP_HAS_PUBLIC_FUNCTOR(y);
SETUP_HAS_PUBLIC_FUNCTOR(z);
SETUP_HAS_PUBLIC_FUNCTOR(w);

SETUP_HAS_PUBLIC_FUNCTOR(X);
SETUP_HAS_PUBLIC_FUNCTOR(Y);
SETUP_HAS_PUBLIC_FUNCTOR(Z);
SETUP_HAS_PUBLIC_FUNCTOR(W);

SETUP_HAS_PUBLIC_DATA_MEMBER(position);
SETUP_HAS_PUBLIC_DATA_MEMBER(orientation);

SETUP_HAS_PUBLIC_DATA_MEMBER(linear);
SETUP_HAS_PUBLIC_DATA_MEMBER(angular);

SETUP_HAS_PUBLIC_DATA_MEMBER(pose);
SETUP_HAS_PUBLIC_DATA_MEMBER(velocity);
SETUP_HAS_PUBLIC_DATA_MEMBER(acceleration);

SETUP_HAS_PUBLIC_DATA_MEMBER(validity);
SETUP_HAS_PUBLIC_DATA_MEMBER(setpoint);

template <class T>
constexpr auto is_1d = (has_public_data_member_x<T>::value);
template <class T>
constexpr auto is_2d = (is_1d<T> && has_public_data_member_y<T>::value);
template <class T>
constexpr auto is_3d = (is_2d<T> && has_public_data_member_z<T>::value);
template <class T>
constexpr auto is_4d = (is_3d<T> && has_public_data_member_w<T>::value);

struct method
{};
struct data_member : public method
{};
struct getter : public method
{};
struct setter : public method
{};

struct name_case
{};
struct lower : public name_case
{};
struct upper : public name_case
{};

template <class T, class = void>
struct x_traits
{
    using method = void;
    using name_case = void;
};
template <class T>
struct x_traits<T, std::enable_if_t<detail::has_public_data_member_x<T>::value>>
{
    using method = data_member;
    using name_case = lower;
};
template <class T>
struct x_traits<T,
                std::enable_if_t<!detail::has_public_data_member_x<T>::value &&
                                 detail::has_public_data_member_X<T>::value>>
{
    using method = data_member;
    using name_case = upper;
};
template <class T>
struct x_traits<T,
                std::enable_if_t<!detail::has_public_data_member_x<T>::value &&
                                 !detail::has_public_data_member_X<T>::value &&
                                 detail::has_public_functor_x<T>::value>>
{
    using method = getter;
    using name_case = lower;
};
template <class T>
struct x_traits<T,
                std::enable_if_t<!detail::has_public_data_member_x<T>::value &&
                                 !detail::has_public_data_member_X<T>::value &&
                                 !detail::has_public_functor_x<T>::value &&
                                 detail::has_public_functor_X<T>::value>>
{
    using method = getter;
    using name_case = upper;
};

template <class T, class = void>
struct y_traits
{
    using method = void;
    using name_case = void;
};
template <class T>
struct y_traits<T, std::enable_if_t<detail::has_public_data_member_y<T>::value>>
{
    using method = data_member;
    using name_case = lower;
};
template <class T>
struct y_traits<T,
                std::enable_if_t<!detail::has_public_data_member_y<T>::value &&
                                 detail::has_public_data_member_Y<T>::value>>
{
    using method = data_member;
    using name_case = upper;
};
template <class T>
struct y_traits<T,
                std::enable_if_t<!detail::has_public_data_member_y<T>::value &&
                                 !detail::has_public_data_member_Y<T>::value &&
                                 detail::has_public_functor_y<T>::value>>
{
    using method = getter;
    using name_case = lower;
};
template <class T>
struct y_traits<T,
                std::enable_if_t<!detail::has_public_data_member_y<T>::value &&
                                 !detail::has_public_data_member_Y<T>::value &&
                                 !detail::has_public_functor_y<T>::value &&
                                 detail::has_public_functor_Y<T>::value>>
{
    using method = getter;
    using name_case = upper;
};

template <class T, class = void>
struct z_traits
{
    using method = void;
    using name_case = void;
};
template <class T>
struct z_traits<T, std::enable_if_t<detail::has_public_data_member_z<T>::value>>
{
    using method = data_member;
    using name_case = lower;
};
template <class T>
struct z_traits<T,
                std::enable_if_t<!detail::has_public_data_member_z<T>::value &&
                                 detail::has_public_data_member_Z<T>::value>>
{
    using method = data_member;
    using name_case = upper;
};
template <class T>
struct z_traits<T,
                std::enable_if_t<!detail::has_public_data_member_z<T>::value &&
                                 !detail::has_public_data_member_Z<T>::value &&
                                 detail::has_public_functor_z<T>::value>>
{
    using method = getter;
    using name_case = lower;
};
template <class T>
struct z_traits<T,
                std::enable_if_t<!detail::has_public_data_member_z<T>::value &&
                                 !detail::has_public_data_member_Z<T>::value &&
                                 !detail::has_public_functor_z<T>::value &&
                                 detail::has_public_functor_Z<T>::value>>
{
    using method = getter;
    using name_case = upper;
};

template <class T, class = void>
struct w_traits
{
    using method = void;
    using name_case = void;
};
template <class T>
struct w_traits<T, std::enable_if_t<detail::has_public_data_member_w<T>::value>>
{
    using method = data_member;
    using name_case = lower;
};
template <class T>
struct w_traits<T,
                std::enable_if_t<!detail::has_public_data_member_w<T>::value &&
                                 detail::has_public_data_member_W<T>::value>>
{
    using method = data_member;
    using name_case = upper;
};
template <class T>
struct w_traits<T,
                std::enable_if_t<!detail::has_public_data_member_w<T>::value &&
                                 !detail::has_public_data_member_W<T>::value &&
                                 detail::has_public_functor_w<T>::value>>
{
    using method = getter;
    using name_case = lower;
};
template <class T>
struct w_traits<T,
                std::enable_if_t<!detail::has_public_data_member_w<T>::value &&
                                 !detail::has_public_data_member_W<T>::value &&
                                 !detail::has_public_functor_w<T>::value &&
                                 detail::has_public_functor_W<T>::value>>
{
    using method = getter;
    using name_case = upper;
};
}  // namespace detail

template <class T>
constexpr auto is_position = detail::is_3d<T>;
template <class T>
constexpr auto is_orientation = detail::is_4d<T>;
template <class T>
constexpr auto is_linear = detail::is_3d<T>;
template <class T>
constexpr auto is_angular = detail::is_3d<T>;

namespace detail {
template <class T>
constexpr auto is_motion = (detail::has_public_data_member_linear<T>::value &&
                            detail::has_public_data_member_angular<T>::value &&
                            is_linear<decltype(std::declval<T>().linear)> &&
                            is_angular<decltype(std::declval<T>().angular)>);
}  // namespace detail

template <class T>
constexpr auto is_pose =
    (detail::has_public_data_member_position<T>::value &&
     detail::has_public_data_member_orientation<T>::value &&
     is_position<decltype(std::declval<T>().position)> &&
     is_orientation<decltype(std::declval<T>().orientation)>);

template <class T>
constexpr auto is_velocity = detail::is_motion<T>;
template <class T>
constexpr auto is_acceleration = detail::is_motion<T>;

template <class T>
constexpr auto is_setpoint =
    (detail::has_public_data_member_pose<T>::value &&
     detail::has_public_data_member_velocity<T>::value &&
     detail::has_public_data_member_acceleration<T>::value &&
     is_pose<decltype(std::declval<T>().pose)> &&
     is_velocity<decltype(std::declval<T>().velocity)> &&
     is_acceleration<decltype(std::declval<T>().acceleration)>);
template <class T>
constexpr auto is_validity = is_setpoint<T>;

template <class T>
constexpr auto is_command =
    (detail::has_public_data_member_setpoint<T>::value &&
     detail::has_public_data_member_validity<T>::value &&
     is_setpoint<decltype(std::declval<T>().setpoint)> &&
     is_validity<decltype(std::declval<T>().validity)>);
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_TYPE_TRAITS_HPP_ */
