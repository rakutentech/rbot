#ifndef _RBOT_UTILS_TEST_MATH_DEADZONE_HPP_
#define _RBOT_UTILS_TEST_MATH_DEADZONE_HPP_

#include "gtest/gtest.h"

#include <rbot_utils/v1/math/deadzone.hpp>

TEST(Deadzone, zero_offset)
{
    using namespace rbot_utils::v1;

    EXPECT_EQ(0, deadzone(0.5, 0.6));
    EXPECT_EQ(0, deadzone(-0.5, 0.6));

    EXPECT_EQ(0.6, deadzone(0.6, 0.5));
    EXPECT_EQ(-0.6, deadzone(-0.6, 0.5));
}

TEST(Deadzone, non_zero_offset)
{
    using namespace rbot_utils::v1;

    EXPECT_EQ(0.7, deadzone(0.7, 0.5, 0.1));
    EXPECT_EQ(-0.7, deadzone(-0.7, 0.5, 0.1));

    EXPECT_EQ(0.25, deadzone(0.7, 0.5, 0.25));
    EXPECT_EQ(-0.7, deadzone(-0.7, 0.5, 0.25));

    EXPECT_EQ(0.7, deadzone(0.7, 0.5, -0.1));
    EXPECT_EQ(-0.7, deadzone(-0.7, 0.5, -0.1));

    EXPECT_EQ(0.7, deadzone(0.7, 0.5, -0.25));
    EXPECT_EQ(-0.25, deadzone(-0.7, 0.5, -0.25));
}
#endif  // _RBOT_UTILS_TEST_MATH_DEADZONE_HPP_
