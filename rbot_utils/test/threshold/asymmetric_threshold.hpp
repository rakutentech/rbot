#ifndef _RBOT_UTILS_TEST_THRESHOLD_ASYMMETRIC_THRESHOLD_HPP_
#define _RBOT_UTILS_TEST_THRESHOLD_ASYMMETRIC_THRESHOLD_HPP_

#include "gtest/gtest.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <rbot_msgs/ControlCommand.h>
#include <rbot_msgs/State.h>

#include <rbot_utils/threshold.hpp>

TEST(AsymmetricThreshold_1d, incomplete_tests)
{
    using namespace geometry_msgs;
    Quaternion a, b;
    EXPECT_TRUE(false);
}

#endif  // _RBOT_UTILS_TEST_THRESHOLD_ASYMMETRIC_THRESHOLD_HPP_
