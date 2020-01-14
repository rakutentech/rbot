#ifndef _RBOT_UTILS_TEST_ALGORITHM_COPY_IF_HPP_
#define _RBOT_UTILS_TEST_ALGORITHM_COPY_IF_HPP_

#include "gtest/gtest.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <rbot_msgs/State.h>

#include <rbot_utils/v1/algorithm/copy_if.hpp>

TEST(Copy_if, copy_true)
{
    int a = 2, b, c = -1;

    EXPECT_NE(a, b);
    rbot_utils::v1::detail::copy_if(b, a, c);
    EXPECT_EQ(a, b);
}

TEST(Copy_if, copy_false)
{
    int a = 2, b, c = 0;

    EXPECT_NE(a, b);
    rbot_utils::v1::detail::copy_if(b, a, c);
    EXPECT_NE(a, b);
}

TEST(Copy_1d_if, copy_true)
{
    geometry_msgs::Quaternion a, b, c;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    c.x = 1;
    c.y = 1;
    c.z = 2;
    c.w = 3;

    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
    rbot_utils::v1::copy_1d_if(b, a, c);
    EXPECT_EQ(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
}

TEST(Copy_1d_if, copy_false)
{
    geometry_msgs::Quaternion a, b, c;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    c.x = 0;
    c.y = 0;
    c.z = 0;
    c.w = 0;

    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
    rbot_utils::v1::copy_1d_if(b, a, c);
    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
}

TEST(Copy_2d_if, copy_true)
{
    geometry_msgs::Quaternion a, b, c;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    c.x = 1;
    c.y = 1;
    c.z = 2;
    c.w = 3;

    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
    rbot_utils::v1::copy_2d_if(b, a, c);
    EXPECT_EQ(a.x, b.x);
    EXPECT_EQ(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
}

TEST(Copy_2d_if, copy_false)
{
    geometry_msgs::Quaternion a, b, c;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    c.x = 0;
    c.y = 0;
    c.z = 0;
    c.w = 0;

    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
    rbot_utils::v1::copy_2d_if(b, a, c);
    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
}

TEST(Copy_3d_if, copy_true)
{
    geometry_msgs::Quaternion a, b, c;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    c.x = 1;
    c.y = 1;
    c.z = 2;
    c.w = 3;

    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
    rbot_utils::v1::copy_3d_if(b, a, c);
    EXPECT_EQ(a.x, b.x);
    EXPECT_EQ(a.y, b.y);
    EXPECT_EQ(a.z, b.z);
    EXPECT_NE(a.w, b.w);
}

TEST(Copy_3d_if, copy_false)
{
    geometry_msgs::Quaternion a, b, c;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    c.x = 0;
    c.y = 0;
    c.z = 0;
    c.w = 0;

    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
    rbot_utils::v1::copy_3d_if(b, a, c);
    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
}

TEST(Copy_4d_if, copy_true)
{
    geometry_msgs::Quaternion a, b, c;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    c.x = 1;
    c.y = 1;
    c.z = 2;
    c.w = 3;

    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
    rbot_utils::v1::copy_4d_if(b, a, c);
    EXPECT_EQ(a.x, b.x);
    EXPECT_EQ(a.y, b.y);
    EXPECT_EQ(a.z, b.z);
    EXPECT_EQ(a.w, b.w);
}

TEST(Copy_4d_if, copy_false)
{
    geometry_msgs::Quaternion a, b, c;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    c.x = 0;
    c.y = 0;
    c.z = 0;
    c.w = 0;

    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
    rbot_utils::v1::copy_4d_if(b, a, c);
    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
}

TEST(Copy_pose_if, copy_true)
{
    geometry_msgs::Pose a, b, c;

    a.position.x = 1;
    a.position.y = 2;
    a.position.z = 3;
    a.orientation.x = 5;
    a.orientation.y = 6;
    a.orientation.z = 7;
    a.orientation.w = 8;

    c.position.x = 2;
    c.position.y = 3;
    c.position.z = 4;
    c.orientation.x = 6;
    c.orientation.y = 7;
    c.orientation.z = 8;
    c.orientation.w = 9;

    EXPECT_NE(a.position.x, b.position.x);
    EXPECT_NE(a.position.y, b.position.y);
    EXPECT_NE(a.position.z, b.position.z);
    EXPECT_NE(a.orientation.x, b.orientation.x);
    EXPECT_NE(a.orientation.y, b.orientation.y);
    EXPECT_NE(a.orientation.z, b.orientation.z);
    EXPECT_NE(a.orientation.w, b.orientation.w);
    rbot_utils::v1::copy_pose_if(b, a, c);
    EXPECT_EQ(a.position.x, b.position.x);
    EXPECT_EQ(a.position.y, b.position.y);
    EXPECT_EQ(a.position.z, b.position.z);
    EXPECT_EQ(a.orientation.x, b.orientation.x);
    EXPECT_EQ(a.orientation.y, b.orientation.y);
    EXPECT_EQ(a.orientation.z, b.orientation.z);
    EXPECT_EQ(a.orientation.w, b.orientation.w);
}

TEST(Copy_pose_if, copy_false)
{
    geometry_msgs::Pose a, b, c;

    a.position.x = 1;
    a.position.y = 2;
    a.position.z = 3;
    a.orientation.x = 5;
    a.orientation.y = 6;
    a.orientation.z = 7;
    a.orientation.w = 8;

    c.position.x = 0;
    c.position.y = 0;
    c.position.z = 0;
    c.orientation.x = 0;
    c.orientation.y = 0;
    c.orientation.z = 0;
    c.orientation.w = 0;

    EXPECT_NE(a.position.x, b.position.x);
    EXPECT_NE(a.position.y, b.position.y);
    EXPECT_NE(a.position.z, b.position.z);
    EXPECT_NE(a.orientation.x, b.orientation.x);
    EXPECT_NE(a.orientation.y, b.orientation.y);
    EXPECT_NE(a.orientation.z, b.orientation.z);
    EXPECT_NE(a.orientation.w, b.orientation.w);
    rbot_utils::v1::copy_pose_if(b, a, c);
    EXPECT_NE(a.position.x, b.position.x);
    EXPECT_NE(a.position.y, b.position.y);
    EXPECT_NE(a.position.z, b.position.z);
    EXPECT_NE(a.orientation.x, b.orientation.x);
    EXPECT_NE(a.orientation.y, b.orientation.y);
    EXPECT_NE(a.orientation.z, b.orientation.z);
    EXPECT_NE(a.orientation.w, b.orientation.w);
}

TEST(Copy_motion_if, copy_true)
{
    geometry_msgs::Twist a, b, c;

    a.linear.x = 1;
    a.linear.y = 2;
    a.linear.z = 3;
    a.angular.x = 5;
    a.angular.y = 6;
    a.angular.z = 7;

    c.linear.x = 2;
    c.linear.y = 3;
    c.linear.z = 4;
    c.angular.x = 6;
    c.angular.y = 7;
    c.angular.z = 8;

    EXPECT_NE(a.linear.x, b.linear.x);
    EXPECT_NE(a.linear.y, b.linear.y);
    EXPECT_NE(a.linear.z, b.linear.z);
    EXPECT_NE(a.angular.x, b.angular.x);
    EXPECT_NE(a.angular.y, b.angular.y);
    EXPECT_NE(a.angular.z, b.angular.z);
    rbot_utils::v1::copy_motion_if(b, a, c);
    EXPECT_EQ(a.linear.x, b.linear.x);
    EXPECT_EQ(a.linear.y, b.linear.y);
    EXPECT_EQ(a.linear.z, b.linear.z);
    EXPECT_EQ(a.angular.x, b.angular.x);
    EXPECT_EQ(a.angular.y, b.angular.y);
    EXPECT_EQ(a.angular.z, b.angular.z);

    b = geometry_msgs::Twist();
    EXPECT_NE(a.linear.x, b.linear.x);
    EXPECT_NE(a.linear.y, b.linear.y);
    EXPECT_NE(a.linear.z, b.linear.z);
    EXPECT_NE(a.angular.x, b.angular.x);
    EXPECT_NE(a.angular.y, b.angular.y);
    EXPECT_NE(a.angular.z, b.angular.z);
    rbot_utils::v1::copy_velocity_if(b, a, c);
    EXPECT_EQ(a.linear.x, b.linear.x);
    EXPECT_EQ(a.linear.y, b.linear.y);
    EXPECT_EQ(a.linear.z, b.linear.z);
    EXPECT_EQ(a.angular.x, b.angular.x);
    EXPECT_EQ(a.angular.y, b.angular.y);
    EXPECT_EQ(a.angular.z, b.angular.z);

    b = geometry_msgs::Twist();
    EXPECT_NE(a.linear.x, b.linear.x);
    EXPECT_NE(a.linear.y, b.linear.y);
    EXPECT_NE(a.linear.z, b.linear.z);
    EXPECT_NE(a.angular.x, b.angular.x);
    EXPECT_NE(a.angular.y, b.angular.y);
    EXPECT_NE(a.angular.z, b.angular.z);
    rbot_utils::v1::copy_acceleration_if(b, a, c);
    EXPECT_EQ(a.linear.x, b.linear.x);
    EXPECT_EQ(a.linear.y, b.linear.y);
    EXPECT_EQ(a.linear.z, b.linear.z);
    EXPECT_EQ(a.angular.x, b.angular.x);
    EXPECT_EQ(a.angular.y, b.angular.y);
    EXPECT_EQ(a.angular.z, b.angular.z);
}

TEST(Copy_motion_if, copy_false)
{
    geometry_msgs::Twist a, b, c;

    a.linear.x = 1;
    a.linear.y = 2;
    a.linear.z = 3;
    a.angular.x = 5;
    a.angular.y = 6;
    a.angular.z = 7;

    c.linear.x = 0;
    c.linear.y = 0;
    c.linear.z = 0;
    c.angular.x = 0;
    c.angular.y = 0;
    c.angular.z = 0;

    EXPECT_NE(a.linear.x, b.linear.x);
    EXPECT_NE(a.linear.y, b.linear.y);
    EXPECT_NE(a.linear.z, b.linear.z);
    EXPECT_NE(a.angular.x, b.angular.x);
    EXPECT_NE(a.angular.y, b.angular.y);
    EXPECT_NE(a.angular.z, b.angular.z);
    rbot_utils::v1::copy_motion_if(b, a, c);
    EXPECT_NE(a.linear.x, b.linear.x);
    EXPECT_NE(a.linear.y, b.linear.y);
    EXPECT_NE(a.linear.z, b.linear.z);
    EXPECT_NE(a.angular.x, b.angular.x);
    EXPECT_NE(a.angular.y, b.angular.y);
    EXPECT_NE(a.angular.z, b.angular.z);

    b = geometry_msgs::Twist();
    EXPECT_NE(a.linear.x, b.linear.x);
    EXPECT_NE(a.linear.y, b.linear.y);
    EXPECT_NE(a.linear.z, b.linear.z);
    EXPECT_NE(a.angular.x, b.angular.x);
    EXPECT_NE(a.angular.y, b.angular.y);
    EXPECT_NE(a.angular.z, b.angular.z);
    rbot_utils::v1::copy_velocity_if(b, a, c);
    EXPECT_NE(a.linear.x, b.linear.x);
    EXPECT_NE(a.linear.y, b.linear.y);
    EXPECT_NE(a.linear.z, b.linear.z);
    EXPECT_NE(a.angular.x, b.angular.x);
    EXPECT_NE(a.angular.y, b.angular.y);
    EXPECT_NE(a.angular.z, b.angular.z);

    b = geometry_msgs::Twist();
    EXPECT_NE(a.linear.x, b.linear.x);
    EXPECT_NE(a.linear.y, b.linear.y);
    EXPECT_NE(a.linear.z, b.linear.z);
    EXPECT_NE(a.angular.x, b.angular.x);
    EXPECT_NE(a.angular.y, b.angular.y);
    EXPECT_NE(a.angular.z, b.angular.z);
    rbot_utils::v1::copy_acceleration_if(b, a, c);
    EXPECT_NE(a.linear.x, b.linear.x);
    EXPECT_NE(a.linear.y, b.linear.y);
    EXPECT_NE(a.linear.z, b.linear.z);
    EXPECT_NE(a.angular.x, b.angular.x);
    EXPECT_NE(a.angular.y, b.angular.y);
    EXPECT_NE(a.angular.z, b.angular.z);
}

TEST(Copy_setpoint_if, copy_true)
{
    rbot_msgs::State a, b, c;

    a.pose.position.x = 1;
    a.pose.position.y = 2;
    a.pose.position.z = 3;
    a.pose.orientation.x = 5;
    a.pose.orientation.y = 6;
    a.pose.orientation.z = 7;
    a.pose.orientation.w = 8;
    a.velocity.linear.x = -1;
    a.velocity.linear.y = -2;
    a.velocity.linear.z = -3;
    a.velocity.angular.x = -5;
    a.velocity.angular.y = -6;
    a.velocity.angular.z = -7;
    a.acceleration.linear.x = 8;
    a.acceleration.linear.y = 9;
    a.acceleration.linear.z = 10;
    a.acceleration.angular.x = -8;
    a.acceleration.angular.y = -9;
    a.acceleration.angular.z = -10;

    c.pose.position.x = 2;
    c.pose.position.y = 3;
    c.pose.position.z = 4;
    c.pose.orientation.x = 6;
    c.pose.orientation.y = 7;
    c.pose.orientation.z = 8;
    c.pose.orientation.w = 9;
    c.velocity.linear.x = -2;
    c.velocity.linear.y = -3;
    c.velocity.linear.z = -4;
    c.velocity.angular.x = -6;
    c.velocity.angular.y = -7;
    c.velocity.angular.z = -8;
    c.acceleration.linear.x = 9;
    c.acceleration.linear.y = 10;
    c.acceleration.linear.z = 11;
    c.acceleration.angular.x = -9;
    c.acceleration.angular.y = -10;
    c.acceleration.angular.z = -11;

    EXPECT_NE(a.pose.position.x, b.pose.position.x);
    EXPECT_NE(a.pose.position.y, b.pose.position.y);
    EXPECT_NE(a.pose.position.z, b.pose.position.z);
    EXPECT_NE(a.pose.orientation.x, b.pose.orientation.x);
    EXPECT_NE(a.pose.orientation.y, b.pose.orientation.y);
    EXPECT_NE(a.pose.orientation.z, b.pose.orientation.z);
    EXPECT_NE(a.pose.orientation.w, b.pose.orientation.w);
    EXPECT_NE(a.velocity.linear.x, b.velocity.linear.x);
    EXPECT_NE(a.velocity.linear.y, b.velocity.linear.y);
    EXPECT_NE(a.velocity.linear.z, b.velocity.linear.z);
    EXPECT_NE(a.velocity.angular.x, b.velocity.angular.x);
    EXPECT_NE(a.velocity.angular.y, b.velocity.angular.y);
    EXPECT_NE(a.velocity.angular.z, b.velocity.angular.z);
    EXPECT_NE(a.acceleration.linear.x, b.acceleration.linear.x);
    EXPECT_NE(a.acceleration.linear.y, b.acceleration.linear.y);
    EXPECT_NE(a.acceleration.linear.z, b.acceleration.linear.z);
    EXPECT_NE(a.acceleration.angular.x, b.acceleration.angular.x);
    EXPECT_NE(a.acceleration.angular.y, b.acceleration.angular.y);
    EXPECT_NE(a.acceleration.angular.z, b.acceleration.angular.z);
    rbot_utils::v1::copy_setpoint_if(b, a, c);
    EXPECT_EQ(a.pose.position.x, b.pose.position.x);
    EXPECT_EQ(a.pose.position.y, b.pose.position.y);
    EXPECT_EQ(a.pose.position.z, b.pose.position.z);
    EXPECT_EQ(a.pose.orientation.x, b.pose.orientation.x);
    EXPECT_EQ(a.pose.orientation.y, b.pose.orientation.y);
    EXPECT_EQ(a.pose.orientation.z, b.pose.orientation.z);
    EXPECT_EQ(a.pose.orientation.w, b.pose.orientation.w);
    EXPECT_EQ(a.velocity.linear.x, b.velocity.linear.x);
    EXPECT_EQ(a.velocity.linear.y, b.velocity.linear.y);
    EXPECT_EQ(a.velocity.linear.z, b.velocity.linear.z);
    EXPECT_EQ(a.velocity.angular.x, b.velocity.angular.x);
    EXPECT_EQ(a.velocity.angular.y, b.velocity.angular.y);
    EXPECT_EQ(a.velocity.angular.z, b.velocity.angular.z);
    EXPECT_EQ(a.acceleration.linear.x, b.acceleration.linear.x);
    EXPECT_EQ(a.acceleration.linear.y, b.acceleration.linear.y);
    EXPECT_EQ(a.acceleration.linear.z, b.acceleration.linear.z);
    EXPECT_EQ(a.acceleration.angular.x, b.acceleration.angular.x);
    EXPECT_EQ(a.acceleration.angular.y, b.acceleration.angular.y);
    EXPECT_EQ(a.acceleration.angular.z, b.acceleration.angular.z);
}

TEST(Copy_setpoint_if, copy_false)
{
    rbot_msgs::State a, b, c;

    a.pose.position.x = 1;
    a.pose.position.y = 2;
    a.pose.position.z = 3;
    a.pose.orientation.x = 5;
    a.pose.orientation.y = 6;
    a.pose.orientation.z = 7;
    a.pose.orientation.w = 8;
    a.velocity.linear.x = -1;
    a.velocity.linear.y = -2;
    a.velocity.linear.z = -3;
    a.velocity.angular.x = -5;
    a.velocity.angular.y = -6;
    a.velocity.angular.z = -7;
    a.acceleration.linear.x = 8;
    a.acceleration.linear.y = 9;
    a.acceleration.linear.z = 10;
    a.acceleration.angular.x = -8;
    a.acceleration.angular.y = -9;
    a.acceleration.angular.z = -10;

    c.pose.position.x = 0;
    c.pose.position.y = 0;
    c.pose.position.z = 0;
    c.pose.orientation.x = 0;
    c.pose.orientation.y = 0;
    c.pose.orientation.z = 0;
    c.pose.orientation.w = 0;
    c.velocity.linear.x = 0;
    c.velocity.linear.y = 0;
    c.velocity.linear.z = 0;
    c.velocity.angular.x = 0;
    c.velocity.angular.y = 0;
    c.velocity.angular.z = 0;
    c.acceleration.linear.x = 0;
    c.acceleration.linear.y = 0;
    c.acceleration.linear.z = 0;
    c.acceleration.angular.x = 0;
    c.acceleration.angular.y = 0;
    c.acceleration.angular.z = 0;

    EXPECT_NE(a.pose.position.x, b.pose.position.x);
    EXPECT_NE(a.pose.position.y, b.pose.position.y);
    EXPECT_NE(a.pose.position.z, b.pose.position.z);
    EXPECT_NE(a.pose.orientation.x, b.pose.orientation.x);
    EXPECT_NE(a.pose.orientation.y, b.pose.orientation.y);
    EXPECT_NE(a.pose.orientation.z, b.pose.orientation.z);
    EXPECT_NE(a.pose.orientation.w, b.pose.orientation.w);
    EXPECT_NE(a.velocity.linear.x, b.velocity.linear.x);
    EXPECT_NE(a.velocity.linear.y, b.velocity.linear.y);
    EXPECT_NE(a.velocity.linear.z, b.velocity.linear.z);
    EXPECT_NE(a.velocity.angular.x, b.velocity.angular.x);
    EXPECT_NE(a.velocity.angular.y, b.velocity.angular.y);
    EXPECT_NE(a.velocity.angular.z, b.velocity.angular.z);
    EXPECT_NE(a.acceleration.linear.x, b.acceleration.linear.x);
    EXPECT_NE(a.acceleration.linear.y, b.acceleration.linear.y);
    EXPECT_NE(a.acceleration.linear.z, b.acceleration.linear.z);
    EXPECT_NE(a.acceleration.angular.x, b.acceleration.angular.x);
    EXPECT_NE(a.acceleration.angular.y, b.acceleration.angular.y);
    EXPECT_NE(a.acceleration.angular.z, b.acceleration.angular.z);
    rbot_utils::v1::copy_setpoint_if(b, a, c);
    EXPECT_NE(a.pose.position.x, b.pose.position.x);
    EXPECT_NE(a.pose.position.y, b.pose.position.y);
    EXPECT_NE(a.pose.position.z, b.pose.position.z);
    EXPECT_NE(a.pose.orientation.x, b.pose.orientation.x);
    EXPECT_NE(a.pose.orientation.y, b.pose.orientation.y);
    EXPECT_NE(a.pose.orientation.z, b.pose.orientation.z);
    EXPECT_NE(a.pose.orientation.w, b.pose.orientation.w);
    EXPECT_NE(a.velocity.linear.x, b.velocity.linear.x);
    EXPECT_NE(a.velocity.linear.y, b.velocity.linear.y);
    EXPECT_NE(a.velocity.linear.z, b.velocity.linear.z);
    EXPECT_NE(a.velocity.angular.x, b.velocity.angular.x);
    EXPECT_NE(a.velocity.angular.y, b.velocity.angular.y);
    EXPECT_NE(a.velocity.angular.z, b.velocity.angular.z);
    EXPECT_NE(a.acceleration.linear.x, b.acceleration.linear.x);
    EXPECT_NE(a.acceleration.linear.y, b.acceleration.linear.y);
    EXPECT_NE(a.acceleration.linear.z, b.acceleration.linear.z);
    EXPECT_NE(a.acceleration.angular.x, b.acceleration.angular.x);
    EXPECT_NE(a.acceleration.angular.y, b.acceleration.angular.y);
    EXPECT_NE(a.acceleration.angular.z, b.acceleration.angular.z);
}
#endif /* ifndef _RBOT_UTILS_TEST_ALGORITHM_COPY_IF_HPP_ */
