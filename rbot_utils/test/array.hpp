#ifndef _RBOT_UTILS_TEST_ARRAY_HPP_
#define _RBOT_UTILS_TEST_ARRAY_HPP_

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <array>
#include <vector>

#include <rbot_utils/array.hpp>

TEST(Array, create)
{
    std::array<int, 4> base = {2, 2, 2, 2};
    auto test = rbot_utils::create_array<int, 4>(2);
    EXPECT_THAT(test, testing::ContainerEq(base));
}

TEST(Array, create_empty)
{
    std::array<int, 0> base;
    auto test = rbot_utils::create_array<int, 0>(2);
    EXPECT_THAT(test, testing::ContainerEq(base));
}

TEST(Array, post_fill_array)
{
    std::array<int, 4> base = {2, 2, 2, 2};
    auto test = rbot_utils::post_fix_array<int, 4, 8>(base, 9);
    EXPECT_THAT(test, testing::ElementsAre(2, 2, 2, 2, 9, 9, 9, 9));
}

TEST(Array, post_fill_array_equal)
{
    std::array<int, 4> base = {2, 2, 2, 2};
    auto test = rbot_utils::post_fix_array<int, 4, 4>(base, 9);
    EXPECT_THAT(test, testing::ContainerEq(base));
}

TEST(Array, pre_fill_array)
{
    std::array<int, 4> base = {2, 2, 2, 2};
    auto test = rbot_utils::pre_fix_array<int, 4, 8>(base, 9);
    EXPECT_THAT(test, testing::ElementsAre(9, 9, 9, 9, 2, 2, 2, 2));
}

TEST(Array, pre_fill_array_equal)
{
    std::array<int, 4> base = {2, 2, 2, 2};
    auto test = rbot_utils::pre_fix_array<int, 4, 4>(base, 9);
    EXPECT_THAT(test, testing::ContainerEq(base));
}
#endif  // _RBOT_UTILS_TEST_ARRAY_HPP_
