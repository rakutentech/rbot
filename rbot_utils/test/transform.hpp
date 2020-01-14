#ifndef _RBOT_UTILS_TEST_TRANSFORM_HPP_
#define _RBOT_UTILS_TEST_TRANSFORM_HPP_

#include "gtest/gtest.h"

#include <cmath>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <rbot_msgs/State.h>

#include <rbot_utils/math.hpp>
#include <rbot_utils/transform.hpp>

TEST(TransformTraits, simpleChecks)
{
    using namespace rbot_utils;
    static_assert(is_position<geometry_msgs::Vector3>,
                  "false negative detected");
    static_assert(is_orientation<geometry_msgs::Quaternion>,
                  "false negative detected");
    static_assert(is_pose<geometry_msgs::Pose>, "false negative detected");
    static_assert(
        is_pose<decltype(std::declval<geometry_msgs::PoseStamped>().pose)>,
        "false negative detected");
}

TEST(GetQuaternion, angleAxis)
{
    rbot_utils::Quaterniond q;
    rbot_utils::Vector3d vec;

    q = rbot_utils::getQuaternion(0, rbot_utils::Vector3d::UnitX());
    EXPECT_DOUBLE_EQ(rbot_utils::Quaterniond(1, 0, 0, 0).w(), q.w());
    EXPECT_EQ(rbot_utils::Quaterniond(1, 0, 0, 0).vec(), q.vec());

    q = rbot_utils::getQuaternion(0, rbot_utils::Vector3d::UnitY());
    EXPECT_DOUBLE_EQ(rbot_utils::Quaterniond(1, 0, 0, 0).w(), q.w());
    EXPECT_EQ(rbot_utils::Quaterniond(1, 0, 0, 0).vec(), q.vec());

    q = rbot_utils::getQuaternion(0, rbot_utils::Vector3d::UnitZ());
    EXPECT_DOUBLE_EQ(rbot_utils::Quaterniond(1, 0, 0, 0).w(), q.w());
    EXPECT_EQ(rbot_utils::Quaterniond(1, 0, 0, 0).vec(), q.vec());

    q = rbot_utils::getQuaternion(rbot_utils::pi,
                                  rbot_utils::Vector3d::UnitX());
    EXPECT_NEAR(rbot_utils::Quaterniond(0, 1, 0, 0).w(), q.w(), 1e-6);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(rbot_utils::Quaterniond(0, 1, 0, 0).vec()[i],
                         q.vec()[i]);
    }

    q = rbot_utils::getQuaternion(rbot_utils::pi,
                                  rbot_utils::Vector3d::UnitY());
    EXPECT_NEAR(rbot_utils::Quaterniond(0, 0, 1, 0).w(), q.w(), 1e-6);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(rbot_utils::Quaterniond(0, 0, 1, 0).vec()[i],
                         q.vec()[i]);
    }

    q = rbot_utils::getQuaternion(rbot_utils::pi,
                                  rbot_utils::Vector3d::UnitZ());
    EXPECT_NEAR(rbot_utils::Quaterniond(0, 0, 0, 1).w(), q.w(), 1e-6);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(rbot_utils::Quaterniond(0, 0, 0, 1).vec()[i],
                         q.vec()[i]);
    }

    q = rbot_utils::getQuaternion(rbot_utils::pi / 2,
                                  rbot_utils::Vector3d::UnitX());
    vec = q.vec();
    vec.normalize();
    EXPECT_NEAR(std::sin(rbot_utils::pi / 4), q.w(), 1e-6);
    EXPECT_EQ(rbot_utils::Vector3d(1, 0, 0), vec);

    q = rbot_utils::getQuaternion(rbot_utils::pi / 2,
                                  rbot_utils::Vector3d::UnitY());
    vec = q.vec();
    vec.normalize();
    EXPECT_NEAR(std::sin(rbot_utils::pi / 4), q.w(), 1e-6);
    EXPECT_EQ(rbot_utils::Vector3d(0, 1, 0), vec);

    q = rbot_utils::getQuaternion(rbot_utils::pi / 2,
                                  rbot_utils::Vector3d::UnitZ());
    vec = q.vec();
    vec.normalize();
    EXPECT_NEAR(std::sin(rbot_utils::pi / 4), q.w(), 1e-6);
    EXPECT_EQ(rbot_utils::Vector3d(0, 0, 1), vec);
}

TEST(GetQuaternion, orientation)
{
    geometry_msgs::Quaternion msg;
    rbot_utils::Quaterniond q;
    rbot_utils::Vector3d vec;

    msg.w = 2;
    msg.x = 3;
    msg.y = 4;
    msg.z = 5;

    q = rbot_utils::getQuaternion(msg, false);
    EXPECT_EQ(msg.w, q.w());
    EXPECT_EQ(msg.x, q.x());
    EXPECT_EQ(msg.y, q.y());
    EXPECT_EQ(msg.z, q.z());

    q = rbot_utils::getQuaternion(msg, true);
    EXPECT_NE(msg.w, q.w());
    EXPECT_NE(msg.x, q.x());
    EXPECT_NE(msg.y, q.y());
    EXPECT_NE(msg.z, q.z());

    msg.w = 1;
    msg.x = 0;
    msg.y = 0;
    msg.z = 0;

    q = rbot_utils::getQuaternion(msg, false);
    EXPECT_EQ(msg.w, q.w());
    EXPECT_EQ(msg.x, q.x());
    EXPECT_EQ(msg.y, q.y());
    EXPECT_EQ(msg.z, q.z());

    q = rbot_utils::getQuaternion(msg, true);
    EXPECT_EQ(msg.w, q.w());
    EXPECT_EQ(msg.x, q.x());
    EXPECT_EQ(msg.y, q.y());
    EXPECT_EQ(msg.z, q.z());
}

TEST(GetTransformMatrix, baseFrame)
{
    geometry_msgs::Pose no_transform;
    no_transform.orientation.w = 1;

    auto t = rbot_utils::getTransformMatrix(no_transform);

    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(0, t.translation()(i));
    }
    for (auto i = 0; i < 3; ++i) {
        for (auto j = 0; j < 3; ++j) {
            EXPECT_DOUBLE_EQ(i == j, t.rotation()(i, j));
        }
    }
}

TEST(GetTransformMatrix, pureInvertedRotationFrame)
{
    geometry_msgs::Pose x_180;
    x_180.orientation.x = 1;

    auto t = rbot_utils::getTransformMatrix(x_180);

    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(0, t.translation()(i));
    }

    for (auto i = 0; i < 3; ++i) {
        for (auto j = 0; j < 3; ++j) {
            if (i == j && i != 0) {
                EXPECT_DOUBLE_EQ(-1, t.rotation()(i, j));
            } else {
                EXPECT_DOUBLE_EQ(i == j, t.rotation()(i, j));
            }
        }
    }

    geometry_msgs::Pose y_180;
    y_180.orientation.y = 1;

    t = rbot_utils::getTransformMatrix(y_180);

    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(0, t.translation()(i));
    }

    for (auto i = 0; i < 3; ++i) {
        for (auto j = 0; j < 3; ++j) {
            if (i == j && i != 1) {
                EXPECT_DOUBLE_EQ(-1, t.rotation()(i, j));
            } else {
                EXPECT_DOUBLE_EQ(i == j, t.rotation()(i, j));
            }
        }
    }

    geometry_msgs::Pose z_180;
    z_180.orientation.z = 1;

    t = rbot_utils::getTransformMatrix(z_180);

    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(0, t.translation()(i));
    }

    for (auto i = 0; i < 3; ++i) {
        for (auto j = 0; j < 3; ++j) {
            if (i == j && i != 2) {
                EXPECT_DOUBLE_EQ(-1, t.rotation()(i, j));
            } else {
                EXPECT_DOUBLE_EQ(i == j, t.rotation()(i, j));
            }
        }
    }
}

TEST(GetTransformMatrix, singleAxisRotationFrame)
{
    // Magic numbers (approx, not actual)
    // Rotation with w = 0.8 and 0.6 respectively around a single axis
    // The axis x, y, z are all 0 except for one which is 0.6 or 0.8
    // (depending on w)
    double rotation_8 = 1.287;
    double rotation_6 = 1.855;

    rbot_utils::Vector3d result, expected;
    rbot_utils::Transform3d t;
    geometry_msgs::Pose arbitary;

    // X-axis
    arbitary.orientation.w = 0.8;
    arbitary.orientation.x = 0.6;
    t = rbot_utils::getTransformMatrix(arbitary);

    result = t * rbot_utils::Vector3d::UnitX();
    expected = rbot_utils::Vector3d(1, 0, 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }
    result = t * rbot_utils::Vector3d::UnitY();
    expected =
        rbot_utils::Vector3d(0, std::cos(rotation_8), std::sin(rotation_8));
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }

    result = t * rbot_utils::Vector3d::UnitZ();
    expected =
        rbot_utils::Vector3d(0, -std::sin(rotation_8), std::cos(rotation_8));
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }

    arbitary.orientation.w = 0.6;
    arbitary.orientation.x = 0.8;
    t = rbot_utils::getTransformMatrix(arbitary);

    result = t * rbot_utils::Vector3d::UnitX();
    expected = rbot_utils::Vector3d(1, 0, 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }
    result = t * rbot_utils::Vector3d::UnitY();
    expected =
        rbot_utils::Vector3d(0, std::cos(rotation_6), std::sin(rotation_6));
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }

    result = t * rbot_utils::Vector3d::UnitZ();
    expected =
        rbot_utils::Vector3d(0, -std::sin(rotation_6), std::cos(rotation_6));
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }

    // Y-axis
    arbitary.orientation.w = 0.8;
    arbitary.orientation.x = 0;
    arbitary.orientation.y = 0.6;
    t = rbot_utils::getTransformMatrix(arbitary);

    result = t * rbot_utils::Vector3d::UnitY();
    expected = rbot_utils::Vector3d(0, 1, 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }
    result = t * rbot_utils::Vector3d::UnitZ();
    expected =
        rbot_utils::Vector3d(std::sin(rotation_8), 0, std::cos(rotation_8));
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }

    result = t * rbot_utils::Vector3d::UnitX();
    expected =
        rbot_utils::Vector3d(std::cos(rotation_8), 0, -std::sin(rotation_8));
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }

    arbitary.orientation.w = 0.6;
    arbitary.orientation.y = 0.8;
    t = rbot_utils::getTransformMatrix(arbitary);

    result = t * rbot_utils::Vector3d::UnitY();
    expected = rbot_utils::Vector3d(0, 1, 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }
    result = t * rbot_utils::Vector3d::UnitZ();
    expected =
        rbot_utils::Vector3d(std::sin(rotation_6), 0, std::cos(rotation_6));
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }

    result = t * rbot_utils::Vector3d::UnitX();
    expected =
        rbot_utils::Vector3d(std::cos(rotation_6), 0, -std::sin(rotation_6));
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }

    // Z-axis
    arbitary.orientation.w = 0.8;
    arbitary.orientation.y = 0;
    arbitary.orientation.z = 0.6;
    t = rbot_utils::getTransformMatrix(arbitary);

    result = t * rbot_utils::Vector3d::UnitZ();
    expected = rbot_utils::Vector3d(0, 0, 1);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }
    result = t * rbot_utils::Vector3d::UnitX();
    expected =
        rbot_utils::Vector3d(std::cos(rotation_8), std::sin(rotation_8), 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }

    result = t * rbot_utils::Vector3d::UnitY();
    expected =
        rbot_utils::Vector3d(-std::sin(rotation_8), std::cos(rotation_8), 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }

    arbitary.orientation.w = 0.6;
    arbitary.orientation.z = 0.8;
    t = rbot_utils::getTransformMatrix(arbitary);

    result = t * rbot_utils::Vector3d::UnitZ();
    expected = rbot_utils::Vector3d(0, 0, 1);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }
    result = t * rbot_utils::Vector3d::UnitX();
    expected =
        rbot_utils::Vector3d(std::cos(rotation_6), std::sin(rotation_6), 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }

    result = t * rbot_utils::Vector3d::UnitY();
    expected =
        rbot_utils::Vector3d(-std::sin(rotation_6), std::cos(rotation_6), 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }
}

TEST(GetTransformMatrix, offsetFrame)
{
    rbot_utils::Vector3d result, expected;
    rbot_utils::Transform3d t;
    geometry_msgs::Pose offset;

    offset.orientation.w = 1;

    // X-axis
    offset.position.x = 4;
    t = rbot_utils::getTransformMatrix(offset);

    result = t * rbot_utils::Vector3d::UnitX();
    expected = rbot_utils::Vector3d(5, 0, 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
    result = t * rbot_utils::Vector3d::UnitY();
    expected = rbot_utils::Vector3d(4, 1, 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
    result = t * rbot_utils::Vector3d::UnitZ();
    expected = rbot_utils::Vector3d(4, 0, 1);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-3);
    }

    // Y-axis
    offset.position.x = 0;
    offset.position.y = 4;
    t = rbot_utils::getTransformMatrix(offset);

    result = t * rbot_utils::Vector3d::UnitX();
    expected = rbot_utils::Vector3d(1, 4, 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
    result = t * rbot_utils::Vector3d::UnitY();
    expected = rbot_utils::Vector3d(0, 5, 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
    result = t * rbot_utils::Vector3d::UnitZ();
    expected = rbot_utils::Vector3d(0, 4, 1);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }

    // Z-axis
    offset.position.y = 0;
    offset.position.z = 4;
    t = rbot_utils::getTransformMatrix(offset);

    result = t * rbot_utils::Vector3d::UnitX();
    expected = rbot_utils::Vector3d(1, 0, 4);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
    result = t * rbot_utils::Vector3d::UnitY();
    expected = rbot_utils::Vector3d(0, 1, 4);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
    result = t * rbot_utils::Vector3d::UnitZ();
    expected = rbot_utils::Vector3d(0, 0, 5);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }

    // Mixed
    offset.position.x = -9;
    offset.position.y = -4;
    offset.position.z = 4;
    t = rbot_utils::getTransformMatrix(offset);

    result = t * rbot_utils::Vector3d::UnitX();
    expected = rbot_utils::Vector3d(-8, -4, 4);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
    result = t * rbot_utils::Vector3d::UnitY();
    expected = rbot_utils::Vector3d(-9, -3, 4);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
    result = t * rbot_utils::Vector3d::UnitZ();
    expected = rbot_utils::Vector3d(-9, -4, 5);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
}

TEST(GetTransformMatrix, mixedFrame)
{
    rbot_utils::Vector3d result, expected;
    rbot_utils::Transform3d t;
    geometry_msgs::Pose offset;

    offset.position.x = 0;
    offset.position.y = 0;
    offset.position.z = 0;

    // Rotate around X axis, then rotate around Y axis
    offset.orientation.w = 0.5;
    offset.orientation.x = 0.5;
    offset.orientation.y = 0.5;
    offset.orientation.z = 0.5;

    t = rbot_utils::getTransformMatrix(offset);

    result = t * rbot_utils::Vector3d::UnitX();
    expected = rbot_utils::Vector3d(0, 1, 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
    result = t * rbot_utils::Vector3d::UnitY();
    expected = rbot_utils::Vector3d(0, 0, 1);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
    result = t * rbot_utils::Vector3d::UnitZ();
    expected = rbot_utils::Vector3d(1, 0, 0);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }

    offset.position.x = 3;
    offset.position.y = 4;
    offset.position.z = 5;

    t = rbot_utils::getTransformMatrix(offset);

    result = t * rbot_utils::Vector3d::UnitX();
    expected = rbot_utils::Vector3d(3, 5, 5);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
    result = t * rbot_utils::Vector3d::UnitY();
    expected = rbot_utils::Vector3d(3, 4, 6);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
    result = t * rbot_utils::Vector3d::UnitZ();
    expected = rbot_utils::Vector3d(4, 4, 5);
    for (auto i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(expected[i], result[i]);
    }
}

TEST(TransformPosition, simpleTest)
{
    geometry_msgs::Pose offset;
    geometry_msgs::Vector3 position, expected;

    offset.position.x = 3;
    offset.position.y = 4;
    offset.position.z = 5;

    // Rotate around X axis, then rotate around Y axis
    offset.orientation.w = 0.5;
    offset.orientation.x = 0.5;
    offset.orientation.y = 0.5;
    offset.orientation.z = 0.5;

    position.x = -3;
    position.y = -4;
    position.z = -5;

    expected.x = -2;
    expected.y = 1;
    expected.z = 1;

    auto result = rbot_utils::transformPosition(offset, position);
    EXPECT_DOUBLE_EQ(expected.x, result.x);
    EXPECT_DOUBLE_EQ(expected.y, result.y);
    EXPECT_DOUBLE_EQ(expected.z, result.z);
}

TEST(TransformPositionInverse, simpleTest)
{
    geometry_msgs::Pose offset;
    geometry_msgs::Vector3 position, expected;

    offset.position.x = 3;
    offset.position.y = 4;
    offset.position.z = 5;

    // Rotate around X axis, then rotate around Y axis
    offset.orientation.w = 0.5;
    offset.orientation.x = 0.5;
    offset.orientation.y = 0.5;
    offset.orientation.z = 0.5;

    expected.x = -3;
    expected.y = -4;
    expected.z = -5;

    position.x = -2;
    position.y = 1;
    position.z = 1;

    auto result = rbot_utils::transformPositionInverse(offset, position);
    EXPECT_DOUBLE_EQ(expected.x, result.x);
    EXPECT_DOUBLE_EQ(expected.y, result.y);
    EXPECT_DOUBLE_EQ(expected.z, result.z);
}

TEST(TransformOrientation, simpleChecks)
{
    geometry_msgs::Quaternion q1, q2, expected;

    expected.w = 1;

    q1.x = 1;
    q2.x = -1;
    auto result = rbot_utils::transformOrientation(q1, q2);
    EXPECT_DOUBLE_EQ(expected.w, result.w);
    EXPECT_DOUBLE_EQ(expected.x, result.x);
    EXPECT_DOUBLE_EQ(expected.y, result.y);
    EXPECT_DOUBLE_EQ(expected.z, result.z);

    q1 = q2 = geometry_msgs::Quaternion();
    q1.y = 1;
    q2.y = -1;
    result = rbot_utils::transformOrientation(q1, q2);
    EXPECT_DOUBLE_EQ(expected.w, result.w);
    EXPECT_DOUBLE_EQ(expected.x, result.x);
    EXPECT_DOUBLE_EQ(expected.y, result.y);
    EXPECT_DOUBLE_EQ(expected.z, result.z);

    q1 = q2 = geometry_msgs::Quaternion();
    q1.z = 1;
    q2.z = -1;
    result = rbot_utils::transformOrientation(q1, q2);
    EXPECT_DOUBLE_EQ(expected.w, result.w);
    EXPECT_DOUBLE_EQ(expected.x, result.x);
    EXPECT_DOUBLE_EQ(expected.y, result.y);
    EXPECT_DOUBLE_EQ(expected.z, result.z);

    q1 = q2 = geometry_msgs::Quaternion();
    q2.w = 1;

    q1.x = 1;
    expected = q1;
    result = rbot_utils::transformOrientation(q1, q2);
    EXPECT_DOUBLE_EQ(expected.w, result.w);
    EXPECT_DOUBLE_EQ(expected.x, result.x);
    EXPECT_DOUBLE_EQ(expected.y, result.y);
    EXPECT_DOUBLE_EQ(expected.z, result.z);

    q1 = geometry_msgs::Quaternion();
    q1.y = 1;
    expected = q1;
    result = rbot_utils::transformOrientation(q1, q2);
    EXPECT_DOUBLE_EQ(expected.w, result.w);
    EXPECT_DOUBLE_EQ(expected.x, result.x);
    EXPECT_DOUBLE_EQ(expected.y, result.y);
    EXPECT_DOUBLE_EQ(expected.z, result.z);

    q1 = geometry_msgs::Quaternion();
    q1.z = 1;
    expected = q1;
    result = rbot_utils::transformOrientation(q1, q2);
    EXPECT_DOUBLE_EQ(expected.w, result.w);
    EXPECT_DOUBLE_EQ(expected.x, result.x);
    EXPECT_DOUBLE_EQ(expected.y, result.y);
    EXPECT_DOUBLE_EQ(expected.z, result.z);

    q1 = geometry_msgs::Quaternion();
    q2.w = -1;

    q1.x = 1;
    expected = q1;
    expected.x *= -1;
    result = rbot_utils::transformOrientation(q1, q2);
    EXPECT_DOUBLE_EQ(expected.w, result.w);
    EXPECT_DOUBLE_EQ(expected.x, result.x);
    EXPECT_DOUBLE_EQ(expected.y, result.y);
    EXPECT_DOUBLE_EQ(expected.z, result.z);

    q1 = geometry_msgs::Quaternion();
    q1.y = 1;
    expected = q1;
    expected.y *= -1;
    result = rbot_utils::transformOrientation(q1, q2);
    EXPECT_DOUBLE_EQ(expected.w, result.w);
    EXPECT_DOUBLE_EQ(expected.x, result.x);
    EXPECT_DOUBLE_EQ(expected.y, result.y);
    EXPECT_DOUBLE_EQ(expected.z, result.z);

    q1 = geometry_msgs::Quaternion();
    q1.z = 1;
    expected = q1;
    expected.z *= -1;
    result = rbot_utils::transformOrientation(q1, q2);
    EXPECT_DOUBLE_EQ(expected.w, result.w);
    EXPECT_DOUBLE_EQ(expected.x, result.x);
    EXPECT_DOUBLE_EQ(expected.y, result.y);
    EXPECT_DOUBLE_EQ(expected.z, result.z);
}

TEST(TransformOrientation, testCases) { EXPECT_TRUE(false); }

TEST(TransformOrientationInvert, simpleChecks) { EXPECT_TRUE(false); }

TEST(TransformOrientationInvert, testCases) { EXPECT_TRUE(false); }

TEST(TransformPose, simpleTest) { EXPECT_TRUE(false); }

TEST(TransformPoseInverse, simpleTest) { EXPECT_TRUE(false); }
#endif  // _RBOT_UTILS_TEST_TRANSFORM_HPP_
