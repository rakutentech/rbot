#ifndef _RBOT_UTILS_TEST_MATH_THRESHOLD_HPP_
#define _RBOT_UTILS_TEST_MATH_THRESHOLD_HPP_

#include "gtest/gtest.h"

#include <rbot_utils/v1/math/threshold.hpp>

TEST(Limit, positive)
{
    using namespace rbot_utils::v1;

    int x, min = 1, max = 9;
    int result = 0;

    x = (min + max) / 2;
    limit(x, result, min, max);
    EXPECT_GE(x, min);
    EXPECT_LE(x, max);
    EXPECT_EQ((min + max) / 2, x);
    EXPECT_EQ(0, result);

    x = min - 1;
    limit(x, result, min, max);
    EXPECT_GE(x, min);
    EXPECT_LE(x, max);
    EXPECT_EQ(min, x);
    EXPECT_EQ(-1, result);

    x = max + 1;
    limit(x, result, min, max);
    EXPECT_GE(x, min);
    EXPECT_LE(x, max);
    EXPECT_EQ(max, x);
    EXPECT_EQ(1, result);
}

TEST(Limit, negative)
{
    using namespace rbot_utils::v1;

    int x, min = -9, max = -1;
    int result = 0;

    x = (min + max) / 2;
    limit(x, result, min, max);
    EXPECT_GE(x, min);
    EXPECT_LE(x, max);
    EXPECT_EQ((min + max) / 2, x);
    EXPECT_EQ(0, result);

    x = min - 1;
    limit(x, result, min, max);
    EXPECT_GE(x, min);
    EXPECT_LE(x, max);
    EXPECT_EQ(min, x);
    EXPECT_EQ(-1, result);

    x = max + 1;
    limit(x, result, min, max);
    EXPECT_GE(x, min);
    EXPECT_LE(x, max);
    EXPECT_EQ(max, x);
    EXPECT_EQ(1, result);
}

TEST(SymmetricThreshold, zero_offset)
{
    using namespace rbot_utils::v1;

    int x, threshold = 6;
    int result = 0;

    x = threshold / 2;
    symmetric_threshold(x, result, threshold);
    EXPECT_GE(x, -threshold);
    EXPECT_LE(x, threshold);
    EXPECT_EQ(threshold / 2, x);
    EXPECT_EQ(0, result);

    x = -threshold - 1;
    symmetric_threshold(x, result, threshold);
    EXPECT_GE(x, -threshold);
    EXPECT_LE(x, threshold);
    EXPECT_EQ(-threshold, x);
    EXPECT_EQ(-1, result);

    x = threshold + 1;
    symmetric_threshold(x, result, threshold);
    EXPECT_GE(x, -threshold);
    EXPECT_LE(x, threshold);
    EXPECT_EQ(threshold, x);
    EXPECT_EQ(1, result);
}

TEST(SymmetricThreshold, non_zero_offset)
{
    using namespace rbot_utils::v1;

    int x, threshold = 6, zero = 4;
    int result = 0;

    x = zero + threshold / 2;
    symmetric_threshold(x, result, threshold, zero);
    EXPECT_GE(x, zero - threshold);
    EXPECT_LE(x, zero + threshold);
    EXPECT_EQ(zero + threshold / 2, x);
    EXPECT_EQ(0, result);

    x = zero - threshold - 1;
    symmetric_threshold(x, result, threshold, zero);
    EXPECT_GE(x, zero - threshold);
    EXPECT_LE(x, zero + threshold);
    EXPECT_EQ(zero - threshold, x);
    EXPECT_EQ(-1, result);

    x = zero + threshold + 1;
    symmetric_threshold(x, result, threshold, zero);
    EXPECT_GE(x, zero - threshold);
    EXPECT_LE(x, zero + threshold);
    EXPECT_EQ(zero + threshold, x);
    EXPECT_EQ(1, result);
}

TEST(AsymmetricThreshold, zero_offset)
{
    using namespace rbot_utils::v1;

    int x, l_thresh = 2, u_thresh = 6;
    int result = 0;

    x = (l_thresh + u_thresh) / 2;
    asymmetric_threshold(x, result, l_thresh, u_thresh);
    EXPECT_GE(x, -l_thresh);
    EXPECT_LE(x, u_thresh);
    EXPECT_EQ((l_thresh + u_thresh) / 2, x);
    EXPECT_EQ(0, result);

    x = -l_thresh - 1;
    asymmetric_threshold(x, result, l_thresh, u_thresh);
    EXPECT_GE(x, -l_thresh);
    EXPECT_LE(x, u_thresh);
    EXPECT_EQ(-l_thresh, x);
    EXPECT_EQ(-1, result);

    x = u_thresh + 1;
    asymmetric_threshold(x, result, l_thresh, u_thresh);
    EXPECT_GE(x, -l_thresh);
    EXPECT_LE(x, u_thresh);
    EXPECT_EQ(u_thresh, x);
    EXPECT_EQ(1, result);
}

TEST(AsymmetricThreshold, non_zero_offset)
{
    using namespace rbot_utils::v1;

    int x, l_thresh = 2, u_thresh = 6, zero = -1;
    int result = 0;

    x = zero + (l_thresh + u_thresh) / 2;
    asymmetric_threshold(x, result, l_thresh, u_thresh, zero);
    EXPECT_GE(x, zero - l_thresh);
    EXPECT_LE(x, zero + u_thresh);
    EXPECT_EQ(zero + (l_thresh + u_thresh) / 2, x);
    EXPECT_EQ(0, result);

    x = zero - l_thresh - 1;
    asymmetric_threshold(x, result, l_thresh, u_thresh, zero);
    EXPECT_GE(x, zero - l_thresh);
    EXPECT_LE(x, zero + u_thresh);
    EXPECT_EQ(zero - l_thresh, x);
    EXPECT_EQ(-1, result);

    x = zero + u_thresh + 1;
    asymmetric_threshold(x, result, l_thresh, u_thresh, zero);
    EXPECT_GE(x, zero - l_thresh);
    EXPECT_LE(x, zero + u_thresh);
    EXPECT_EQ(zero + u_thresh, x);
    EXPECT_EQ(1, result);
}
#endif  // _RBOT_UTILS_TEST_MATH_THRESHOLD_HPP_
