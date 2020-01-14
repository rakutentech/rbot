#ifndef _RBOT_UTILS_TEST_INTROSPECTION_HPP_
#define _RBOT_UTILS_TEST_INTROSPECTION_HPP_

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <rbot_utils/macro.hpp>

namespace rbot_utils_test {
SETUP_HAS_PUBLIC_DATA_MEMBER(x);
SETUP_HAS_PUBLIC_DATA_MEMBER(y);
SETUP_HAS_PUBLIC_FUNCTOR(a);
SETUP_HAS_PUBLIC_FUNCTOR(b);
}  // namespace rbot_utils_test

TEST(Introspection, has_public_data_member)
{
    struct A
    {
        int x;
    };
    struct B : public A
    {
        int y;
    };
    static_assert(rbot_utils_test::has_public_data_member_x<A>::value,
                  "introspection toolchain faiure: false negative");
    static_assert(rbot_utils_test::has_public_data_member_y<B>::value,
                  "introspection toolchain failure: false negative");

    static_assert(rbot_utils_test::has_public_data_member_x<B>::value,
                  "introspection toolchain failure: false negative");
}

TEST(Introspection, has_no_public_data_member)
{
    struct A
    {
        int x;
    };
    struct B : protected A
    {
        int y;
    };
    static_assert(!rbot_utils_test::has_public_data_member_y<A>::value,
                  "introspection toolchain failure: false positive");
#ifdef __clang__
    // this is a bug in GCC
    static_assert(!rbot_utils_test::has_public_data_member_x<B>::value,
                  "introspection toolchain failure: false positive");
#endif
}

TEST(Introspection, has_public_functor)
{
    struct A
    {
        int a() { return 42; }
    };
    struct B : public A
    {
        int b() { return 42; }
    };
    static_assert(rbot_utils_test::has_public_functor_a<A>::value,
                  "introspection toolchain faiure: false negative");
    static_assert(rbot_utils_test::has_public_functor_b<B>::value,
                  "introspection toolchain failure: false negative");

    static_assert(rbot_utils_test::has_public_functor_a<B>::value,
                  "introspection toolchain failure: false negative");
}

TEST(Introspection, has_no_public_functor)
{
    struct A
    {
        int a() { return 42; }
    };
    struct B : protected A
    {
        int b() { return 42; }
    };
    static_assert(!rbot_utils_test::has_public_functor_b<A>::value,
                  "introspection toolchain failure: false positive");
#ifdef __clang__
    // this is a bug in GCC
    static_assert(!rbot_utils_test::has_public_functor_a<B>::value,
                  "introspection toolchain failure: false positive");
#endif
}
#endif  // _RBOT_UTILS_TEST_INTROSPECTION_HPP_
