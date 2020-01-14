#ifndef _RBOT_UTILS_TEST_ALGORITHM_COPY_HPP_
#define _RBOT_UTILS_TEST_ALGORITHM_COPY_HPP_

#include "gtest/gtest.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <rbot_msgs/State.h>

#include <rbot_utils/v1/algorithm/copy.hpp>

namespace {
struct test_4d
{
    int& X() { return _x; }
    int& Y() { return _y; }
    int& Z() { return _z; }
    int& W() { return _w; }
    int X() const { return _x; }
    int Y() const { return _y; }
    int Z() const { return _z; }
    int W() const { return _w; }

  private:
    int _x = 0, _y = 0, _z = 0, _w = 0;
};
}  // namespace

TEST(Copy_1d, field)
{
    geometry_msgs::Quaternion a, b;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
    rbot_utils::v1::copy_1d(b, a);
    EXPECT_EQ(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
}

TEST(Copy_2d, field)
{
    geometry_msgs::Quaternion a, b;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
    rbot_utils::v1::copy_2d(b, a);
    EXPECT_EQ(a.x, b.x);
    EXPECT_EQ(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
}

TEST(Copy_3d, field)
{
    geometry_msgs::Quaternion a, b;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
    rbot_utils::v1::copy_3d(b, a);
    EXPECT_EQ(a.x, b.x);
    EXPECT_EQ(a.y, b.y);
    EXPECT_EQ(a.z, b.z);
    EXPECT_NE(a.w, b.w);
}

TEST(Copy_4d, field)
{
    geometry_msgs::Quaternion a, b;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    EXPECT_NE(a.x, b.x);
    EXPECT_NE(a.y, b.y);
    EXPECT_NE(a.z, b.z);
    EXPECT_NE(a.w, b.w);
    rbot_utils::v1::copy_4d(b, a);
    EXPECT_EQ(a.x, b.x);
    EXPECT_EQ(a.y, b.y);
    EXPECT_EQ(a.z, b.z);
    EXPECT_EQ(a.w, b.w);
}

TEST(Copy_1d, set)
{
    geometry_msgs::Quaternion a;
    test_4d b;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    EXPECT_NE(a.x, b.X());
    EXPECT_NE(a.y, b.Y());
    EXPECT_NE(a.z, b.Z());
    EXPECT_NE(a.w, b.W());
    rbot_utils::v1::copy_1d(b, a);
    EXPECT_EQ(a.x, b.X());
    EXPECT_NE(a.y, b.Y());
    EXPECT_NE(a.z, b.Z());
    EXPECT_NE(a.w, b.W());
}

TEST(Copy_2d, set)
{
    geometry_msgs::Quaternion a;
    test_4d b;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    EXPECT_NE(a.x, b.X());
    EXPECT_NE(a.y, b.Y());
    EXPECT_NE(a.z, b.Z());
    EXPECT_NE(a.w, b.W());
    rbot_utils::v1::copy_2d(b, a);
    EXPECT_EQ(a.x, b.X());
    EXPECT_EQ(a.y, b.Y());
    EXPECT_NE(a.z, b.Z());
    EXPECT_NE(a.w, b.W());
}

TEST(Copy_3d, set)
{
    geometry_msgs::Quaternion a;
    test_4d b;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    EXPECT_NE(a.x, b.X());
    EXPECT_NE(a.y, b.Y());
    EXPECT_NE(a.z, b.Z());
    EXPECT_NE(a.w, b.W());
    rbot_utils::v1::copy_3d(b, a);
    EXPECT_EQ(a.x, b.X());
    EXPECT_EQ(a.y, b.Y());
    EXPECT_EQ(a.z, b.Z());
    EXPECT_NE(a.w, b.W());
}

TEST(Copy_4d, set)
{
    geometry_msgs::Quaternion a;
    test_4d b;

    a.x = 1;
    a.y = 2;
    a.z = 3;
    a.w = 4;

    EXPECT_NE(a.x, b.X());
    EXPECT_NE(a.y, b.Y());
    EXPECT_NE(a.z, b.Z());
    EXPECT_NE(a.w, b.W());
    rbot_utils::v1::copy_4d(b, a);
    EXPECT_EQ(a.x, b.X());
    EXPECT_EQ(a.y, b.Y());
    EXPECT_EQ(a.z, b.Z());
    EXPECT_EQ(a.w, b.W());
}

TEST(Copy_1d, get)
{
    geometry_msgs::Quaternion b;
    test_4d a;

    a.X() = 1;
    a.Y() = 2;
    a.Z() = 3;
    a.W() = 4;

    EXPECT_NE(b.x, a.X());
    EXPECT_NE(b.y, a.Y());
    EXPECT_NE(b.z, a.Z());
    EXPECT_NE(b.w, a.W());
    rbot_utils::v1::copy_1d(b, a);
    EXPECT_EQ(b.x, a.X());
    EXPECT_NE(b.y, a.Y());
    EXPECT_NE(b.z, a.Z());
    EXPECT_NE(b.w, a.W());
}

TEST(Copy_2d, get)
{
    geometry_msgs::Quaternion b;
    test_4d a;

    a.X() = 1;
    a.Y() = 2;
    a.Z() = 3;
    a.W() = 4;

    EXPECT_NE(b.x, a.X());
    EXPECT_NE(b.y, a.Y());
    EXPECT_NE(b.z, a.Z());
    EXPECT_NE(b.w, a.W());
    rbot_utils::v1::copy_2d(b, a);
    EXPECT_EQ(b.x, a.X());
    EXPECT_EQ(b.y, a.Y());
    EXPECT_NE(b.z, a.Z());
    EXPECT_NE(b.w, a.W());
}

TEST(Copy_3d, get)
{
    geometry_msgs::Quaternion b;
    test_4d a;

    a.X() = 1;
    a.Y() = 2;
    a.Z() = 3;
    a.W() = 4;

    EXPECT_NE(b.x, a.X());
    EXPECT_NE(b.y, a.Y());
    EXPECT_NE(b.z, a.Z());
    EXPECT_NE(b.w, a.W());
    rbot_utils::v1::copy_3d(b, a);
    EXPECT_EQ(b.x, a.X());
    EXPECT_EQ(b.y, a.Y());
    EXPECT_EQ(b.z, a.Z());
    EXPECT_NE(b.w, a.W());
}

TEST(Copy_4d, get)
{
    geometry_msgs::Quaternion b;
    test_4d a;

    a.X() = 1;
    a.Y() = 2;
    a.Z() = 3;
    a.W() = 4;

    EXPECT_NE(b.x, a.X());
    EXPECT_NE(b.y, a.Y());
    EXPECT_NE(b.z, a.Z());
    EXPECT_NE(b.w, a.W());
    rbot_utils::v1::copy_4d(b, a);
    EXPECT_EQ(b.x, a.X());
    EXPECT_EQ(b.y, a.Y());
    EXPECT_EQ(b.z, a.Z());
    EXPECT_EQ(b.w, a.W());
}

TEST(Copy_1d, get_set)
{
    test_4d a, b;

    a.X() = 1;
    a.Y() = 2;
    a.Z() = 3;
    a.W() = 4;

    EXPECT_NE(a.X(), b.X());
    EXPECT_NE(a.Y(), b.Y());
    EXPECT_NE(a.Z(), b.Z());
    EXPECT_NE(a.W(), b.W());
    rbot_utils::v1::copy_1d(b, a);
    EXPECT_EQ(a.X(), b.X());
    EXPECT_NE(a.Y(), b.Y());
    EXPECT_NE(a.Z(), b.Z());
    EXPECT_NE(a.W(), b.W());
}

TEST(Copy_2d, get_set)
{
    test_4d a, b;

    a.X() = 1;
    a.Y() = 2;
    a.Z() = 3;
    a.W() = 4;

    EXPECT_NE(a.X(), b.X());
    EXPECT_NE(a.Y(), b.Y());
    EXPECT_NE(a.Z(), b.Z());
    EXPECT_NE(a.W(), b.W());
    rbot_utils::v1::copy_2d(b, a);
    EXPECT_EQ(a.X(), b.X());
    EXPECT_EQ(a.Y(), b.Y());
    EXPECT_NE(a.Z(), b.Z());
    EXPECT_NE(a.W(), b.W());
}

TEST(Copy_3d, get_set)
{
    test_4d a, b;

    a.X() = 1;
    a.Y() = 2;
    a.Z() = 3;
    a.W() = 4;

    EXPECT_NE(a.X(), b.X());
    EXPECT_NE(a.Y(), b.Y());
    EXPECT_NE(a.Z(), b.Z());
    EXPECT_NE(a.W(), b.W());
    rbot_utils::v1::copy_3d(b, a);
    EXPECT_EQ(a.X(), b.X());
    EXPECT_EQ(a.Y(), b.Y());
    EXPECT_EQ(a.Z(), b.Z());
    EXPECT_NE(a.W(), b.W());
}

TEST(Copy_4d, get_set)
{
    test_4d a, b;

    a.X() = 1;
    a.Y() = 2;
    a.Z() = 3;
    a.W() = 4;

    EXPECT_NE(a.X(), b.X());
    EXPECT_NE(a.Y(), b.Y());
    EXPECT_NE(a.Z(), b.Z());
    EXPECT_NE(a.W(), b.W());
    rbot_utils::v1::copy_4d(b, a);
    EXPECT_EQ(a.X(), b.X());
    EXPECT_EQ(a.Y(), b.Y());
    EXPECT_EQ(a.Z(), b.Z());
    EXPECT_EQ(a.W(), b.W());
}

TEST(Copy, pose)
{
    geometry_msgs::Pose a, b;

    a.position.x = 1;
    a.position.y = 2;
    a.position.z = 3;
    a.orientation.x = 5;
    a.orientation.y = 6;
    a.orientation.z = 7;
    a.orientation.w = 8;

    EXPECT_NE(a.position.x, b.position.x);
    EXPECT_NE(a.position.y, b.position.y);
    EXPECT_NE(a.position.z, b.position.z);
    EXPECT_NE(a.orientation.x, b.orientation.x);
    EXPECT_NE(a.orientation.y, b.orientation.y);
    EXPECT_NE(a.orientation.z, b.orientation.z);
    EXPECT_NE(a.orientation.w, b.orientation.w);
    rbot_utils::v1::copy_pose(b, a);
    EXPECT_EQ(a.position.x, b.position.x);
    EXPECT_EQ(a.position.y, b.position.y);
    EXPECT_EQ(a.position.z, b.position.z);
    EXPECT_EQ(a.orientation.x, b.orientation.x);
    EXPECT_EQ(a.orientation.y, b.orientation.y);
    EXPECT_EQ(a.orientation.z, b.orientation.z);
    EXPECT_EQ(a.orientation.w, b.orientation.w);
}

TEST(Copy, motion)
{
    geometry_msgs::Twist a, b;

    a.linear.x = 1;
    a.linear.y = 2;
    a.linear.z = 3;
    a.angular.x = 5;
    a.angular.y = 6;
    a.angular.z = 7;

    EXPECT_NE(a.linear.x, b.linear.x);
    EXPECT_NE(a.linear.y, b.linear.y);
    EXPECT_NE(a.linear.z, b.linear.z);
    EXPECT_NE(a.angular.x, b.angular.x);
    EXPECT_NE(a.angular.y, b.angular.y);
    EXPECT_NE(a.angular.z, b.angular.z);
    rbot_utils::v1::copy_motion(b, a);
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
    rbot_utils::v1::copy_velocity(b, a);
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
    rbot_utils::v1::copy_acceleration(b, a);
    EXPECT_EQ(a.linear.x, b.linear.x);
    EXPECT_EQ(a.linear.y, b.linear.y);
    EXPECT_EQ(a.linear.z, b.linear.z);
    EXPECT_EQ(a.angular.x, b.angular.x);
    EXPECT_EQ(a.angular.y, b.angular.y);
    EXPECT_EQ(a.angular.z, b.angular.z);
}

TEST(Copy, setpoint)
{
    rbot_msgs::State a, b;

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
    rbot_utils::v1::copy_setpoint(b, a);
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
#endif /* ifndef _RBOT_UTILS_TEST_ALGORITHM_COPY_HPP_ */
