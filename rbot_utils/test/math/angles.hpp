#ifndef _RBOT_UTILS_TEST_MATH_ANGLES_HPP_
#define _RBOT_UTILS_TEST_MATH_ANGLES_HPP_

#include "gtest/gtest.h"

#include <rbot_utils/v1/math/angles.hpp>

TEST(Angles, deg2rad)
{
    using namespace rbot_utils::v1;

    EXPECT_EQ(0., deg2rad(0.));

    EXPECT_EQ(pi / 2, deg2rad(90.));
    EXPECT_EQ(pi, deg2rad(180.));
    EXPECT_EQ(3 * pi / 2, deg2rad(270.));
    EXPECT_EQ(10 * pi, deg2rad(1800.));

    EXPECT_EQ(-1 / 2. * pi, deg2rad(-90.));
    EXPECT_EQ(-1 * pi, deg2rad(-180.));
    EXPECT_EQ(-3 / 2. * pi, deg2rad(-270.));
    EXPECT_EQ(-10 * pi, deg2rad(-1800.));
}

TEST(Angles, rad2deg)
{
    using namespace rbot_utils::v1;

    EXPECT_EQ(0., rad2deg(0.));

    EXPECT_EQ(90., rad2deg(pi / 2));
    EXPECT_EQ(180., rad2deg(pi));
    EXPECT_EQ(270., rad2deg(3 / 2. * pi));
    EXPECT_EQ(1800., rad2deg(10 * pi));

    EXPECT_EQ(-90., rad2deg(-1 / 2. * pi));
    EXPECT_EQ(-180., rad2deg(-1 * pi));
    EXPECT_EQ(-270., rad2deg(-3 / 2. * pi));
    EXPECT_EQ(-1800., rad2deg(-10 * pi));
}

TEST(Angles, normalize_angle_direction)
{
    using namespace rbot_utils::v1;

    double epsilon = 1e-6;
    EXPECT_NEAR(0, normalize_angle_direction(0.), epsilon);

    EXPECT_NEAR(6, normalize_angle_direction(6.), epsilon);
    EXPECT_NEAR(7 - 2 * pi, normalize_angle_direction(7.), epsilon);
    EXPECT_NEAR(9 - 2 * pi, normalize_angle_direction(9.), epsilon);

    EXPECT_NEAR(-6, normalize_angle_direction(-6.), epsilon);
    EXPECT_NEAR(-7 + 2 * pi, normalize_angle_direction(-7.), epsilon);
    EXPECT_NEAR(-9 + 2 * pi, normalize_angle_direction(-9.), epsilon);
}

TEST(Angles, normalize_angle_positive)
{
    using namespace rbot_utils::v1;

    double epsilon = 1e-6;
    EXPECT_NEAR(0, normalize_angle_positive(0.), epsilon);

    EXPECT_NEAR(6, normalize_angle_positive(6.), epsilon);
    EXPECT_NEAR(7 - 2 * pi, normalize_angle_positive(7.), epsilon);
    EXPECT_NEAR(9 - 2 * pi, normalize_angle_positive(9.), epsilon);

    EXPECT_NEAR(2 * pi - 3, normalize_angle_positive(-3.), epsilon);
    EXPECT_NEAR(2 * pi - 6, normalize_angle_positive(-6.), epsilon);
    EXPECT_NEAR(-9 + 4 * pi, normalize_angle_positive(-9.), epsilon);
}

TEST(Angles, normalize_angle)
{
    using namespace rbot_utils::v1;

    double epsilon = 1e-6;
    EXPECT_NEAR(0, normalize_angle(0.), epsilon);
    EXPECT_NEAR(3, normalize_angle(3.), epsilon);
    EXPECT_NEAR(4 - 2 * pi, normalize_angle(4.), epsilon);

    EXPECT_NEAR(-3, normalize_angle(-3.), epsilon);
    EXPECT_NEAR(-4 + 2 * pi, normalize_angle(-4.), epsilon);
}

TEST(Angles, incomplete_tests) { EXPECT_TRUE(false); }
#endif /* ifndef _RBOT_UTILS_TEST_MATH_ANGLES_HPP_ */
