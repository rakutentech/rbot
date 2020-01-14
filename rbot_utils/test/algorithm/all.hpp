#ifndef _RBOT_UTILS_TEST_ALGORITHM_ALL_HPP_
#define _RBOT_UTILS_TEST_ALGORITHM_ALL_HPP_

#include "gtest/gtest.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <rbot_msgs/State.h>

#include <rbot_utils/v1/algorithm/all.hpp>

TEST(All, d1)
{
    geometry_msgs::Quaternion a;

    a.x = 0;
    EXPECT_FALSE(rbot_utils::v1::all_1d(a));

    a.x = 1;
    EXPECT_TRUE(rbot_utils::v1::all_1d(a));
}

TEST(All, d2)
{
    geometry_msgs::Quaternion a;

    a.x = 0;
    a.y = 0;
    EXPECT_FALSE(rbot_utils::v1::all_2d(a));

    a.x = 0;
    a.y = 1;
    EXPECT_FALSE(rbot_utils::v1::all_2d(a));

    a.x = 1;
    a.y = 0;
    EXPECT_FALSE(rbot_utils::v1::all_2d(a));

    a.x = 1;
    a.y = 1;
    EXPECT_TRUE(rbot_utils::v1::all_2d(a));
}

TEST(All, d3)
{
    geometry_msgs::Quaternion a;

    a.x = 0;
    a.y = 0;
    a.z = 0;
    EXPECT_FALSE(rbot_utils::v1::all_3d(a));

    a.x = 0;
    a.y = 0;
    a.z = 1;
    EXPECT_FALSE(rbot_utils::v1::all_3d(a));

    a.x = 0;
    a.y = 1;
    a.z = 0;
    EXPECT_FALSE(rbot_utils::v1::all_3d(a));

    a.x = 0;
    a.y = 1;
    a.z = 1;
    EXPECT_FALSE(rbot_utils::v1::all_3d(a));

    a.x = 1;
    a.y = 0;
    a.z = 0;
    EXPECT_FALSE(rbot_utils::v1::all_3d(a));

    a.x = 1;
    a.y = 0;
    a.z = 1;
    EXPECT_FALSE(rbot_utils::v1::all_3d(a));

    a.x = 1;
    a.y = 1;
    a.z = 0;
    EXPECT_FALSE(rbot_utils::v1::all_3d(a));

    a.x = 1;
    a.y = 1;
    a.z = 1;
    EXPECT_TRUE(rbot_utils::v1::all_3d(a));
}

TEST(All, d4)
{
    geometry_msgs::Quaternion a;

    a.x = 0;
    a.y = 0;
    a.z = 0;
    a.w = 0;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 0;
    a.y = 0;
    a.z = 0;
    a.w = 1;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 0;
    a.y = 0;
    a.z = 1;
    a.w = 0;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 0;
    a.y = 0;
    a.z = 1;
    a.w = 1;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 0;
    a.y = 1;
    a.z = 0;
    a.w = 0;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 0;
    a.y = 1;
    a.z = 0;
    a.w = 1;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 0;
    a.y = 1;
    a.z = 0;
    a.w = 1;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 0;
    a.y = 1;
    a.z = 1;
    a.w = 0;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 0;
    a.y = 1;
    a.z = 1;
    a.w = 1;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 1;
    a.y = 0;
    a.z = 0;
    a.w = 0;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 1;
    a.y = 0;
    a.z = 0;
    a.w = 1;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 1;
    a.y = 0;
    a.z = 1;
    a.w = 0;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 1;
    a.y = 0;
    a.z = 1;
    a.w = 1;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 1;
    a.y = 1;
    a.z = 0;
    a.w = 0;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 1;
    a.y = 1;
    a.z = 0;
    a.w = 1;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 1;
    a.y = 1;
    a.z = 0;
    a.w = 1;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 1;
    a.y = 1;
    a.z = 1;
    a.w = 0;
    EXPECT_FALSE(rbot_utils::v1::all_4d(a));

    a.x = 1;
    a.y = 1;
    a.z = 1;
    a.w = 1;
    EXPECT_TRUE(rbot_utils::v1::all_4d(a));
}

TEST(All, pose)
{
    geometry_msgs::Pose a;

    a.position.x = 0;
    a.position.y = 1;
    a.position.z = 1;
    a.orientation.x = 0;
    a.orientation.y = 1;
    a.orientation.z = 1;
    a.orientation.w = 1;
    EXPECT_FALSE(rbot_utils::v1::all_pose(a));

    a.position.x = 1;
    a.position.y = 1;
    a.position.z = 1;
    a.orientation.x = 0;
    a.orientation.y = 1;
    a.orientation.z = 1;
    a.orientation.w = 1;
    EXPECT_FALSE(rbot_utils::v1::all_pose(a));

    a.position.x = 0;
    a.position.y = 1;
    a.position.z = 1;
    a.orientation.x = 1;
    a.orientation.y = 1;
    a.orientation.z = 1;
    a.orientation.w = 1;
    EXPECT_FALSE(rbot_utils::v1::all_pose(a));

    a.position.x = 1;
    a.position.y = 1;
    a.position.z = 1;
    a.orientation.x = 1;
    a.orientation.y = 1;
    a.orientation.z = 1;
    a.orientation.w = 1;
    EXPECT_TRUE(rbot_utils::v1::all_pose(a));
}

TEST(All, motion)
{
    geometry_msgs::Twist a;

    a.linear.x = 0;
    a.linear.y = 1;
    a.linear.z = 1;
    a.angular.x = 0;
    a.angular.y = 1;
    a.angular.z = 1;
    EXPECT_FALSE(rbot_utils::v1::all_motion(a));
    EXPECT_FALSE(rbot_utils::v1::all_velocity(a));
    EXPECT_FALSE(rbot_utils::v1::all_acceleration(a));

    a.linear.x = 0;
    a.linear.y = 1;
    a.linear.z = 1;
    a.angular.x = 1;
    a.angular.y = 1;
    a.angular.z = 1;
    EXPECT_FALSE(rbot_utils::v1::all_motion(a));
    EXPECT_FALSE(rbot_utils::v1::all_velocity(a));
    EXPECT_FALSE(rbot_utils::v1::all_acceleration(a));

    a.linear.x = 1;
    a.linear.y = 1;
    a.linear.z = 1;
    a.angular.x = 0;
    a.angular.y = 1;
    a.angular.z = 1;
    EXPECT_FALSE(rbot_utils::v1::all_motion(a));
    EXPECT_FALSE(rbot_utils::v1::all_velocity(a));
    EXPECT_FALSE(rbot_utils::v1::all_acceleration(a));

    a.linear.x = 1;
    a.linear.y = 1;
    a.linear.z = 1;
    a.angular.x = 1;
    a.angular.y = 1;
    a.angular.z = 1;
    EXPECT_TRUE(rbot_utils::v1::all_motion(a));
    EXPECT_TRUE(rbot_utils::v1::all_velocity(a));
    EXPECT_TRUE(rbot_utils::v1::all_acceleration(a));
}

TEST(All, setpoint)
{
    rbot_msgs::State a;

    a.pose.position.x = 0;
    a.pose.position.y = 1;
    a.pose.position.z = 1;
    a.pose.orientation.x = 1;
    a.pose.orientation.y = 1;
    a.pose.orientation.z = 1;
    a.pose.orientation.w = 1;
    a.velocity.linear.x = 0;
    a.velocity.linear.y = 1;
    a.velocity.linear.z = 1;
    a.velocity.angular.x = 1;
    a.velocity.angular.y = 1;
    a.velocity.angular.z = 1;
    a.acceleration.linear.x = 0;
    a.acceleration.linear.y = 1;
    a.acceleration.linear.z = 1;
    a.acceleration.angular.x = 1;
    a.acceleration.angular.y = 1;
    a.acceleration.angular.z = 1;
    EXPECT_FALSE(rbot_utils::v1::all_setpoint(a));

    a.pose.position.x = 0;
    a.pose.position.y = 1;
    a.pose.position.z = 1;
    a.pose.orientation.x = 1;
    a.pose.orientation.y = 1;
    a.pose.orientation.z = 1;
    a.pose.orientation.w = 1;
    a.velocity.linear.x = 0;
    a.velocity.linear.y = 1;
    a.velocity.linear.z = 1;
    a.velocity.angular.x = 1;
    a.velocity.angular.y = 1;
    a.velocity.angular.z = 1;
    a.acceleration.linear.x = 1;
    a.acceleration.linear.y = 1;
    a.acceleration.linear.z = 1;
    a.acceleration.angular.x = 1;
    a.acceleration.angular.y = 1;
    a.acceleration.angular.z = 1;
    EXPECT_FALSE(rbot_utils::v1::all_setpoint(a));

    a.pose.position.x = 0;
    a.pose.position.y = 1;
    a.pose.position.z = 1;
    a.pose.orientation.x = 1;
    a.pose.orientation.y = 1;
    a.pose.orientation.z = 1;
    a.pose.orientation.w = 1;
    a.velocity.linear.x = 1;
    a.velocity.linear.y = 1;
    a.velocity.linear.z = 1;
    a.velocity.angular.x = 1;
    a.velocity.angular.y = 1;
    a.velocity.angular.z = 1;
    a.acceleration.linear.x = 0;
    a.acceleration.linear.y = 1;
    a.acceleration.linear.z = 1;
    a.acceleration.angular.x = 1;
    a.acceleration.angular.y = 1;
    a.acceleration.angular.z = 1;
    EXPECT_FALSE(rbot_utils::v1::all_setpoint(a));

    a.pose.position.x = 0;
    a.pose.position.y = 1;
    a.pose.position.z = 1;
    a.pose.orientation.x = 1;
    a.pose.orientation.y = 1;
    a.pose.orientation.z = 1;
    a.pose.orientation.w = 1;
    a.velocity.linear.x = 1;
    a.velocity.linear.y = 1;
    a.velocity.linear.z = 1;
    a.velocity.angular.x = 1;
    a.velocity.angular.y = 1;
    a.velocity.angular.z = 1;
    a.acceleration.linear.x = 1;
    a.acceleration.linear.y = 1;
    a.acceleration.linear.z = 1;
    a.acceleration.angular.x = 1;
    a.acceleration.angular.y = 1;
    a.acceleration.angular.z = 1;
    EXPECT_FALSE(rbot_utils::v1::all_setpoint(a));

    a.pose.position.x = 1;
    a.pose.position.y = 1;
    a.pose.position.z = 1;
    a.pose.orientation.x = 1;
    a.pose.orientation.y = 1;
    a.pose.orientation.z = 1;
    a.pose.orientation.w = 1;
    a.velocity.linear.x = 0;
    a.velocity.linear.y = 1;
    a.velocity.linear.z = 1;
    a.velocity.angular.x = 1;
    a.velocity.angular.y = 1;
    a.velocity.angular.z = 1;
    a.acceleration.linear.x = 0;
    a.acceleration.linear.y = 1;
    a.acceleration.linear.z = 1;
    a.acceleration.angular.x = 1;
    a.acceleration.angular.y = 1;
    a.acceleration.angular.z = 1;
    EXPECT_FALSE(rbot_utils::v1::all_setpoint(a));

    a.pose.position.x = 1;
    a.pose.position.y = 1;
    a.pose.position.z = 1;
    a.pose.orientation.x = 1;
    a.pose.orientation.y = 1;
    a.pose.orientation.z = 1;
    a.pose.orientation.w = 1;
    a.velocity.linear.x = 0;
    a.velocity.linear.y = 1;
    a.velocity.linear.z = 1;
    a.velocity.angular.x = 1;
    a.velocity.angular.y = 1;
    a.velocity.angular.z = 1;
    a.acceleration.linear.x = 1;
    a.acceleration.linear.y = 1;
    a.acceleration.linear.z = 1;
    a.acceleration.angular.x = 1;
    a.acceleration.angular.y = 1;
    a.acceleration.angular.z = 1;
    EXPECT_FALSE(rbot_utils::v1::all_setpoint(a));

    a.pose.position.x = 1;
    a.pose.position.y = 1;
    a.pose.position.z = 1;
    a.pose.orientation.x = 1;
    a.pose.orientation.y = 1;
    a.pose.orientation.z = 1;
    a.pose.orientation.w = 1;
    a.velocity.linear.x = 1;
    a.velocity.linear.y = 1;
    a.velocity.linear.z = 1;
    a.velocity.angular.x = 1;
    a.velocity.angular.y = 1;
    a.velocity.angular.z = 1;
    a.acceleration.linear.x = 0;
    a.acceleration.linear.y = 1;
    a.acceleration.linear.z = 1;
    a.acceleration.angular.x = 1;
    a.acceleration.angular.y = 1;
    a.acceleration.angular.z = 1;
    EXPECT_FALSE(rbot_utils::v1::all_setpoint(a));

    a.pose.position.x = 1;
    a.pose.position.y = 1;
    a.pose.position.z = 1;
    a.pose.orientation.x = 1;
    a.pose.orientation.y = 1;
    a.pose.orientation.z = 1;
    a.pose.orientation.w = 1;
    a.velocity.linear.x = 1;
    a.velocity.linear.y = 1;
    a.velocity.linear.z = 1;
    a.velocity.angular.x = 1;
    a.velocity.angular.y = 1;
    a.velocity.angular.z = 1;
    a.acceleration.linear.x = 1;
    a.acceleration.linear.y = 1;
    a.acceleration.linear.z = 1;
    a.acceleration.angular.x = 1;
    a.acceleration.angular.y = 1;
    a.acceleration.angular.z = 1;
    EXPECT_TRUE(rbot_utils::v1::all_setpoint(a));
}
#endif  // _RBOT_UTILS_TEST_ALGORITHM_ALL_HPP_
