#ifndef _RBOT_UTILS_TEST_ALGORITHM_ANY_HPP_
#define _RBOT_UTILS_TEST_ALGORITHM_ANY_HPP_

#include "gtest/gtest.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <rbot_msgs/State.h>

#include <rbot_utils/v1/algorithm/any.hpp>

TEST(Any, d1)
{
    using namespace geometry_msgs;
    geometry_msgs::Quaternion a;

    a.x = 0;
    EXPECT_FALSE(rbot_utils::v1::any_1d(a));

    a.x = 1;
    EXPECT_TRUE(rbot_utils::v1::any_1d(a));
}

TEST(Any, d2)
{
    using namespace geometry_msgs;
    geometry_msgs::Quaternion a;

    a.x = 0;
    a.y = 0;
    EXPECT_FALSE(rbot_utils::v1::any_2d(a));

    a.x = 0;
    a.y = 1;
    EXPECT_TRUE(rbot_utils::v1::any_2d(a));

    a.x = 1;
    a.y = 0;
    EXPECT_TRUE(rbot_utils::v1::any_2d(a));

    a.x = 1;
    a.y = 1;
    EXPECT_TRUE(rbot_utils::v1::any_2d(a));
}

TEST(Any, d3)
{
    using namespace geometry_msgs;
    geometry_msgs::Quaternion a;

    a.x = 0;
    a.y = 0;
    a.z = 0;
    EXPECT_FALSE(rbot_utils::v1::any_3d(a));

    a.x = 0;
    a.y = 0;
    a.z = 1;
    EXPECT_TRUE(rbot_utils::v1::any_3d(a));

    a.x = 0;
    a.y = 1;
    a.z = 0;
    EXPECT_TRUE(rbot_utils::v1::any_3d(a));

    a.x = 0;
    a.y = 1;
    a.z = 1;
    EXPECT_TRUE(rbot_utils::v1::any_3d(a));

    a.x = 1;
    a.y = 0;
    a.z = 0;
    EXPECT_TRUE(rbot_utils::v1::any_3d(a));

    a.x = 1;
    a.y = 0;
    a.z = 1;
    EXPECT_TRUE(rbot_utils::v1::any_3d(a));

    a.x = 1;
    a.y = 1;
    a.z = 0;
    EXPECT_TRUE(rbot_utils::v1::any_3d(a));

    a.x = 1;
    a.y = 1;
    a.z = 1;
    EXPECT_TRUE(rbot_utils::v1::any_3d(a));
}

TEST(Any, d4)
{
    using namespace geometry_msgs;
    geometry_msgs::Quaternion a;

    a.x = 0;
    a.y = 0;
    a.z = 0;
    a.w = 0;
    EXPECT_FALSE(rbot_utils::v1::any_4d(a));

    a.x = 0;
    a.y = 0;
    a.z = 0;
    a.w = 1;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 0;
    a.y = 0;
    a.z = 1;
    a.w = 0;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 0;
    a.y = 0;
    a.z = 1;
    a.w = 1;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 0;
    a.y = 1;
    a.z = 0;
    a.w = 0;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 0;
    a.y = 1;
    a.z = 0;
    a.w = 1;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 0;
    a.y = 1;
    a.z = 0;
    a.w = 1;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 0;
    a.y = 1;
    a.z = 1;
    a.w = 0;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 0;
    a.y = 1;
    a.z = 1;
    a.w = 1;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 1;
    a.y = 0;
    a.z = 0;
    a.w = 0;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 1;
    a.y = 0;
    a.z = 0;
    a.w = 1;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 1;
    a.y = 0;
    a.z = 1;
    a.w = 0;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 1;
    a.y = 0;
    a.z = 1;
    a.w = 1;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 1;
    a.y = 1;
    a.z = 0;
    a.w = 0;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 1;
    a.y = 1;
    a.z = 0;
    a.w = 1;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 1;
    a.y = 1;
    a.z = 0;
    a.w = 1;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 1;
    a.y = 1;
    a.z = 1;
    a.w = 0;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));

    a.x = 1;
    a.y = 1;
    a.z = 1;
    a.w = 1;
    EXPECT_TRUE(rbot_utils::v1::any_4d(a));
}

TEST(Any, pose)
{
    geometry_msgs::Pose a;

    a.position.x = 1;
    a.position.y = 0;
    a.position.z = 0;
    a.orientation.x = 1;
    a.orientation.y = 0;
    a.orientation.z = 0;
    a.orientation.w = 0;
    EXPECT_TRUE(rbot_utils::v1::any_pose(a));

    a.position.x = 0;
    a.position.y = 0;
    a.position.z = 0;
    a.orientation.x = 1;
    a.orientation.y = 0;
    a.orientation.z = 0;
    a.orientation.w = 0;
    EXPECT_TRUE(rbot_utils::v1::any_pose(a));

    a.position.x = 1;
    a.position.y = 0;
    a.position.z = 0;
    a.orientation.x = 0;
    a.orientation.y = 0;
    a.orientation.z = 0;
    a.orientation.w = 0;
    EXPECT_TRUE(rbot_utils::v1::any_pose(a));

    a.position.x = 0;
    a.position.y = 0;
    a.position.z = 0;
    a.orientation.x = 0;
    a.orientation.y = 0;
    a.orientation.z = 0;
    a.orientation.w = 0;
    EXPECT_FALSE(rbot_utils::v1::any_pose(a));
}

TEST(Any, motion)
{
    geometry_msgs::Twist a;

    a.linear.x = 1;
    a.linear.y = 0;
    a.linear.z = 0;
    a.angular.x = 1;
    a.angular.y = 0;
    a.angular.z = 0;
    EXPECT_TRUE(rbot_utils::v1::any_motion(a));
    EXPECT_TRUE(rbot_utils::v1::any_velocity(a));
    EXPECT_TRUE(rbot_utils::v1::any_acceleration(a));

    a.linear.x = 1;
    a.linear.y = 0;
    a.linear.z = 0;
    a.angular.x = 0;
    a.angular.y = 0;
    a.angular.z = 0;
    EXPECT_TRUE(rbot_utils::v1::any_motion(a));
    EXPECT_TRUE(rbot_utils::v1::any_velocity(a));
    EXPECT_TRUE(rbot_utils::v1::any_acceleration(a));

    a.linear.x = 0;
    a.linear.y = 0;
    a.linear.z = 0;
    a.angular.x = 1;
    a.angular.y = 0;
    a.angular.z = 0;
    EXPECT_TRUE(rbot_utils::v1::any_motion(a));
    EXPECT_TRUE(rbot_utils::v1::any_velocity(a));
    EXPECT_TRUE(rbot_utils::v1::any_acceleration(a));

    a.linear.x = 0;
    a.linear.y = 0;
    a.linear.z = 0;
    a.angular.x = 0;
    a.angular.y = 0;
    a.angular.z = 0;
    EXPECT_FALSE(rbot_utils::v1::any_motion(a));
    EXPECT_FALSE(rbot_utils::v1::any_velocity(a));
    EXPECT_FALSE(rbot_utils::v1::any_acceleration(a));
}

TEST(Any, setpoint)
{
    rbot_msgs::State a;

    a.pose.position.x = 1;
    a.pose.position.y = 0;
    a.pose.position.z = 0;
    a.pose.orientation.x = 0;
    a.pose.orientation.y = 0;
    a.pose.orientation.z = 0;
    a.pose.orientation.w = 0;
    a.velocity.linear.x = 1;
    a.velocity.linear.y = 0;
    a.velocity.linear.z = 0;
    a.velocity.angular.x = 0;
    a.velocity.angular.y = 0;
    a.velocity.angular.z = 0;
    a.acceleration.linear.x = 1;
    a.acceleration.linear.y = 0;
    a.acceleration.linear.z = 0;
    a.acceleration.angular.x = 0;
    a.acceleration.angular.y = 0;
    a.acceleration.angular.z = 0;
    EXPECT_TRUE(rbot_utils::v1::any_setpoint(a));

    a.pose.position.x = 1;
    a.pose.position.y = 0;
    a.pose.position.z = 0;
    a.pose.orientation.x = 0;
    a.pose.orientation.y = 0;
    a.pose.orientation.z = 0;
    a.pose.orientation.w = 0;
    a.velocity.linear.x = 1;
    a.velocity.linear.y = 0;
    a.velocity.linear.z = 0;
    a.velocity.angular.x = 0;
    a.velocity.angular.y = 0;
    a.velocity.angular.z = 0;
    a.acceleration.linear.x = 0;
    a.acceleration.linear.y = 0;
    a.acceleration.linear.z = 0;
    a.acceleration.angular.x = 0;
    a.acceleration.angular.y = 0;
    a.acceleration.angular.z = 0;
    EXPECT_TRUE(rbot_utils::v1::any_setpoint(a));

    a.pose.position.x = 1;
    a.pose.position.y = 0;
    a.pose.position.z = 0;
    a.pose.orientation.x = 0;
    a.pose.orientation.y = 0;
    a.pose.orientation.z = 0;
    a.pose.orientation.w = 0;
    a.velocity.linear.x = 0;
    a.velocity.linear.y = 0;
    a.velocity.linear.z = 0;
    a.velocity.angular.x = 0;
    a.velocity.angular.y = 0;
    a.velocity.angular.z = 0;
    a.acceleration.linear.x = 1;
    a.acceleration.linear.y = 0;
    a.acceleration.linear.z = 0;
    a.acceleration.angular.x = 0;
    a.acceleration.angular.y = 0;
    a.acceleration.angular.z = 0;
    EXPECT_TRUE(rbot_utils::v1::any_setpoint(a));

    a.pose.position.x = 1;
    a.pose.position.y = 0;
    a.pose.position.z = 0;
    a.pose.orientation.x = 0;
    a.pose.orientation.y = 0;
    a.pose.orientation.z = 0;
    a.pose.orientation.w = 0;
    a.velocity.linear.x = 0;
    a.velocity.linear.y = 0;
    a.velocity.linear.z = 0;
    a.velocity.angular.x = 0;
    a.velocity.angular.y = 0;
    a.velocity.angular.z = 0;
    a.acceleration.linear.x = 0;
    a.acceleration.linear.y = 0;
    a.acceleration.linear.z = 0;
    a.acceleration.angular.x = 0;
    a.acceleration.angular.y = 0;
    a.acceleration.angular.z = 0;
    EXPECT_TRUE(rbot_utils::v1::any_setpoint(a));

    a.pose.position.x = 0;
    a.pose.position.y = 0;
    a.pose.position.z = 0;
    a.pose.orientation.x = 0;
    a.pose.orientation.y = 0;
    a.pose.orientation.z = 0;
    a.pose.orientation.w = 0;
    a.velocity.linear.x = 1;
    a.velocity.linear.y = 0;
    a.velocity.linear.z = 0;
    a.velocity.angular.x = 0;
    a.velocity.angular.y = 0;
    a.velocity.angular.z = 0;
    a.acceleration.linear.x = 1;
    a.acceleration.linear.y = 0;
    a.acceleration.linear.z = 0;
    a.acceleration.angular.x = 0;
    a.acceleration.angular.y = 0;
    a.acceleration.angular.z = 0;
    EXPECT_TRUE(rbot_utils::v1::any_setpoint(a));

    a.pose.position.x = 0;
    a.pose.position.y = 0;
    a.pose.position.z = 0;
    a.pose.orientation.x = 0;
    a.pose.orientation.y = 0;
    a.pose.orientation.z = 0;
    a.pose.orientation.w = 0;
    a.velocity.linear.x = 1;
    a.velocity.linear.y = 0;
    a.velocity.linear.z = 0;
    a.velocity.angular.x = 0;
    a.velocity.angular.y = 0;
    a.velocity.angular.z = 0;
    a.acceleration.linear.x = 0;
    a.acceleration.linear.y = 0;
    a.acceleration.linear.z = 0;
    a.acceleration.angular.x = 0;
    a.acceleration.angular.y = 0;
    a.acceleration.angular.z = 0;
    EXPECT_TRUE(rbot_utils::v1::any_setpoint(a));

    a.pose.position.x = 0;
    a.pose.position.y = 0;
    a.pose.position.z = 0;
    a.pose.orientation.x = 0;
    a.pose.orientation.y = 0;
    a.pose.orientation.z = 0;
    a.pose.orientation.w = 0;
    a.velocity.linear.x = 0;
    a.velocity.linear.y = 0;
    a.velocity.linear.z = 0;
    a.velocity.angular.x = 0;
    a.velocity.angular.y = 0;
    a.velocity.angular.z = 0;
    a.acceleration.linear.x = 1;
    a.acceleration.linear.y = 0;
    a.acceleration.linear.z = 0;
    a.acceleration.angular.x = 0;
    a.acceleration.angular.y = 0;
    a.acceleration.angular.z = 0;
    EXPECT_TRUE(rbot_utils::v1::any_setpoint(a));

    a.pose.position.x = 0;
    a.pose.position.y = 0;
    a.pose.position.z = 0;
    a.pose.orientation.x = 0;
    a.pose.orientation.y = 0;
    a.pose.orientation.z = 0;
    a.pose.orientation.w = 0;
    a.velocity.linear.x = 0;
    a.velocity.linear.y = 0;
    a.velocity.linear.z = 0;
    a.velocity.angular.x = 0;
    a.velocity.angular.y = 0;
    a.velocity.angular.z = 0;
    a.acceleration.linear.x = 0;
    a.acceleration.linear.y = 0;
    a.acceleration.linear.z = 0;
    a.acceleration.angular.x = 0;
    a.acceleration.angular.y = 0;
    a.acceleration.angular.z = 0;
    EXPECT_FALSE(rbot_utils::v1::any_setpoint(a));
}
#endif  // _RBOT_UTILS_TEST_ALGORITHM_ANY_HPP_
