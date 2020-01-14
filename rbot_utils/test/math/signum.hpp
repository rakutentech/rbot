#ifndef _RBOT_UTILS_TEST_MATH_SIGNUM_HPP_
#define _RBOT_UTILS_TEST_MATH_SIGNUM_HPP_

#include "gtest/gtest.h"

#include <rbot_utils/v1/math/signum.hpp>

TEST(Signum, zero)
{
    using namespace rbot_utils::v1;

    EXPECT_EQ(0, signum(0));
    EXPECT_EQ(0, signum(std::size_t(0)));
    EXPECT_EQ(0, signum(-0.0));
    EXPECT_EQ(0, signum(+0.0));
}

TEST(Signum, positive)
{
    using namespace rbot_utils::v1;

    EXPECT_EQ(1, signum(1));
    EXPECT_EQ(1, signum(std::size_t(2)));
    EXPECT_EQ(1, signum(3.0));
}

TEST(Signum, negative)
{
    using namespace rbot_utils::v1;

    EXPECT_EQ(-1, signum(-1));
    EXPECT_EQ(1, signum(-std::size_t(2)));
    EXPECT_EQ(-1, signum(-3.0));
}
#endif  // _RBOT_UTILS_TEST_MATH_SIGNUM_HPP_
