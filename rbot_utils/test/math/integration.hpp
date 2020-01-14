#ifndef _RBOT_UTILS_TEST_MATH_INTEGRATION_HPP_
#define _RBOT_UTILS_TEST_MATH_INTEGRATION_HPP_

#include "gtest/gtest.h"

#include <algorithm>

#include <rbot_utils/v1/math/integration.hpp>

TEST(PriorLinear, zero)
{
    using namespace rbot_utils::v1;

    EXPECT_DOUBLE_EQ(0, prior_linear_quadrature(0, 1, 2, 3));
    EXPECT_DOUBLE_EQ(0, prior_linear_quadrature(0, -1, 2, -3));
    EXPECT_DOUBLE_EQ(0, prior_linear_quadrature(0, 1, -2, 3));
}

TEST(PriorLinear, simple)
{
    using namespace rbot_utils::v1;

    EXPECT_DOUBLE_EQ(2, prior_linear_quadrature(1., 1, 2., 3));
    EXPECT_DOUBLE_EQ(-2, prior_linear_quadrature(1., -1, 2., -3));
    EXPECT_DOUBLE_EQ(2, prior_linear_quadrature(1., 1, -2., 3));
}

TEST(PosteriorLinear, zero)
{
    using namespace rbot_utils::v1;

    EXPECT_DOUBLE_EQ(0, posterior_linear_quadrature(3, 1, 0, 2));
    EXPECT_DOUBLE_EQ(0, posterior_linear_quadrature(-3, 1, 0, 2));
    EXPECT_DOUBLE_EQ(0, posterior_linear_quadrature(3, -1, 0, -2));
}

TEST(PosteriorLinear, simple)
{
    using namespace rbot_utils::v1;

    EXPECT_DOUBLE_EQ(2, posterior_linear_quadrature(2., 1, 1., 3));
    EXPECT_DOUBLE_EQ(-2, posterior_linear_quadrature(2., -1, 1., -3));
    EXPECT_DOUBLE_EQ(2, posterior_linear_quadrature(-2., 1, 1., 3));
}

TEST(CenterLinear, zero)
{
    using namespace rbot_utils::v1;

    EXPECT_DOUBLE_EQ(0, center_linear_quadrature(2, 1, -2, 3));
    EXPECT_DOUBLE_EQ(0, center_linear_quadrature(2, 3, -2, 1));
    EXPECT_DOUBLE_EQ(0, center_linear_quadrature(5, 1, -5, 3));
}

TEST(CenterLinear, simple)
{
    using namespace rbot_utils::v1;

    EXPECT_DOUBLE_EQ(2, center_linear_quadrature(2., 1, 0., 3));
    EXPECT_DOUBLE_EQ(-2, center_linear_quadrature(2., -1, 0., -3));
    EXPECT_DOUBLE_EQ(2, center_linear_quadrature(0., 1, 2., 3));
}

TEST(Quadratic, delta)
{
    using namespace rbot_utils::detail;

    EXPECT_DOUBLE_EQ(0, quadratic_quadrature(0., 1, 1, 1));
    EXPECT_DOUBLE_EQ(0, quadratic_quadrature(0., -1, -1, -1));

    EXPECT_DOUBLE_EQ(3, quadratic_quadrature(1., 3, 2, 1));

    EXPECT_DOUBLE_EQ(1, quadratic_quadrature(1., -3, 2, 1));
    EXPECT_DOUBLE_EQ(1, quadratic_quadrature(1., 3, -2, 1));
    EXPECT_DOUBLE_EQ(1, quadratic_quadrature(1., 3, 2, -1));

    EXPECT_DOUBLE_EQ(-1, quadratic_quadrature(1., -3, -2, 1));
    EXPECT_DOUBLE_EQ(-1, quadratic_quadrature(1., 3, -2, -1));
    EXPECT_DOUBLE_EQ(-1, quadratic_quadrature(1., -3, 2, -1));

    EXPECT_DOUBLE_EQ(-3, quadratic_quadrature(1., -3, -2, -1));

    EXPECT_DOUBLE_EQ(14, quadratic_quadrature(2., 3, 2, 1));

    EXPECT_DOUBLE_EQ(-2, quadratic_quadrature(2., -3, 2, 1));
    EXPECT_DOUBLE_EQ(6, quadratic_quadrature(2., 3, -2, 1));
    EXPECT_DOUBLE_EQ(10, quadratic_quadrature(2., 3, 2, -1));

    EXPECT_DOUBLE_EQ(-10, quadratic_quadrature(2., -3, -2, 1));
    EXPECT_DOUBLE_EQ(2, quadratic_quadrature(2., 3, -2, -1));
    EXPECT_DOUBLE_EQ(-6, quadratic_quadrature(2., -3, 2, -1));

    EXPECT_DOUBLE_EQ(-14, quadratic_quadrature(2., -3, -2, -1));
}

TEST(Quadratic, first)
{
    using namespace rbot_utils::detail;
    double a, b, c;

    // EXPECT_DOUBLE_EQ(1.5, quadratic_quadrature(1., 1, 2., 2, a, b, c));
    quadratic_quadrature(1., 1, 2., 2, a, b, c);
    EXPECT_DOUBLE_EQ(0, 2 * 1 * a + b);
    EXPECT_DOUBLE_EQ(1, a * 1 * 1 + b * 1 + c);
    EXPECT_DOUBLE_EQ(2, a * 2 * 2 + b * 2 + c);

    // EXPECT_DOUBLE_EQ(2., quadratic_quadrature(1., 1, 3., 2, a, b, c));
    quadratic_quadrature(1., 1, 3., 2, a, b, c);
    EXPECT_DOUBLE_EQ(0, 2 * 1 * a + b);
    EXPECT_DOUBLE_EQ(1, a * 1 * 1 + b * 1 + c);
    EXPECT_DOUBLE_EQ(3, a * 2 * 2 + b * 2 + c);

    // EXPECT_DOUBLE_EQ(1., quadratic_quadrature(1., 1, 1., 2, a, b, c));
    quadratic_quadrature(1., 1, 1., 2, a, b, c);
    EXPECT_DOUBLE_EQ(0, 2 * 1 * a + b);
    EXPECT_DOUBLE_EQ(1, a * 1 * 1 + b * 1 + c);
    EXPECT_DOUBLE_EQ(1, a * 2 * 2 + b * 2 + c);
}

TEST(Quadratic, second)
{
    using namespace rbot_utils::detail;
    double a, b, c, area, old_a, old_b, old_c;

    double x1, y1, x2, y2;

    area = 5;
    old_a = a = 0;
    old_b = b = 0;
    old_c = c = 1;
    x1 = 1;
    y1 = 1;
    x2 = 2;
    y2 = 2;
    EXPECT_DOUBLE_EQ(
        1 + 1 / 3.,
        quadratic_quadrature(area, y1, x1, y2, x2, a, b, c, std::false_type()));
    EXPECT_DOUBLE_EQ(6 + 1 / 3., area);
    EXPECT_DOUBLE_EQ(y1, old_a * x1 * x1 + old_b * x1 + old_c);
    EXPECT_DOUBLE_EQ(y1, a * x1 * x1 + b * x1 + c);
    EXPECT_DOUBLE_EQ(y2, a * x2 * x2 + b * x2 + c);
    EXPECT_DOUBLE_EQ(2 * old_a * x1 + old_b, 2 * a * x1 + b);

    old_a = a = 0;
    old_b = b = -2;
    old_c = c = 6;
    x1 = 2;
    y1 = 2;
    x2 = 5;
    y2 = 4;
    quadratic_quadrature(area, y1, x1, y2, x2, a, b, c, std::false_type());
    EXPECT_DOUBLE_EQ(y1, old_a * x1 * x1 + old_b * x1 + old_c);
    EXPECT_DOUBLE_EQ(y1, a * x1 * x1 + b * x1 + c);
    EXPECT_DOUBLE_EQ(y2, a * x2 * x2 + b * x2 + c);
    EXPECT_DOUBLE_EQ(2 * old_a * x1 + old_b, 2 * a * x1 + b);

    old_a = a = 0;
    old_b = b = 2;
    old_c = c = -2;
    x1 = 2;
    y1 = 2;
    x2 = 5;
    y2 = 4;
    quadratic_quadrature(area, y1, x1, y2, x2, a, b, c, std::false_type());
    EXPECT_DOUBLE_EQ(y1, old_a * x1 * x1 + old_b * x1 + old_c);
    EXPECT_DOUBLE_EQ(y1, a * x1 * x1 + b * x1 + c);
    EXPECT_NEAR(y2, a * x2 * x2 + b * x2 + c, 0.001);
    EXPECT_DOUBLE_EQ(2 * old_a * x1 + old_b, 2 * a * x1 + b);
}

TEST(Quadratic, third)
{
    using namespace rbot_utils::detail;
    double a, b, c, area, old_a, old_b, old_c;

    double x1, y1, x2, y2;

    area = 5;
    old_a = a = 0;
    old_b = b = 0;
    old_c = c = 1;
    x1 = 1;
    y1 = 1;
    x2 = 2;
    y2 = 2;
    quadratic_quadrature(area, y1, x1, y2, x2, a, b, c, std::true_type());
    EXPECT_DOUBLE_EQ(y1, old_a * x1 * x1 + old_b * x1 + old_c);
    EXPECT_DOUBLE_EQ(y1, a * x1 * x1 + b * x1 + c);
    EXPECT_DOUBLE_EQ(y2, a * x2 * x2 + b * x2 + c);
    EXPECT_DOUBLE_EQ(2 * old_a * x1 + old_b, 2 * a * x1 + b);

    old_a = a = 0;
    old_b = b = -2;
    old_c = c = 6;
    x1 = 2;
    y1 = 2;
    x2 = 5;
    y2 = 4;
    quadratic_quadrature(area, y1, x1, y2, x2, a, b, c, std::true_type());
    EXPECT_DOUBLE_EQ(y1, old_a * x1 * x1 + old_b * x1 + old_c);
    EXPECT_DOUBLE_EQ(y1, a * x1 * x1 + b * x1 + c);
    EXPECT_DOUBLE_EQ(y2, a * x2 * x2 + b * x2 + c);
    EXPECT_DOUBLE_EQ(2 * old_a * x1 + old_b, 2 * a * x1 + b);

    old_a = a = 0;
    old_b = b = 2;
    old_c = c = -2;
    x1 = 2;
    y1 = 2;
    x2 = 5;
    y2 = 4;
    quadratic_quadrature(area, y1, x1, y2, x2, a, b, c, std::true_type());
    EXPECT_DOUBLE_EQ(y1, old_a * x1 * x1 + old_b * x1 + old_c);
    EXPECT_DOUBLE_EQ(y1, a * x1 * x1 + b * x1 + c);
    EXPECT_NEAR(y2, a * x2 * x2 + b * x2 + c, 0.001);
    EXPECT_DOUBLE_EQ(2 * old_a * x1 + old_b, 2 * a * x1 + b);

    std::swap(old_a, a);
    std::swap(old_b, b);
    std::swap(old_c, c);
    std::swap(x1, x2);
    std::swap(y1, y2);
    x2 = 6;
    y2 = 7;
    quadratic_quadrature(area, y1, x1, y2, x2, a, b, c, std::true_type());
    EXPECT_DOUBLE_EQ(y1, old_a * x1 * x1 + old_b * x1 + old_c);
    EXPECT_DOUBLE_EQ(y1, a * x1 * x1 + b * x1 + c);
    EXPECT_DOUBLE_EQ(y2, a * x2 * x2 + b * x2 + c);
    EXPECT_DOUBLE_EQ(2 * old_a * x1 + old_b, 2 * a * x1 + b);

    std::swap(old_a, a);
    std::swap(old_b, b);
    std::swap(old_c, c);
    std::swap(x1, x2);
    std::swap(y1, y2);
    x2 = 7.3;
    y2 = 4;
    quadratic_quadrature(area, y1, x1, y2, x2, a, b, c, std::true_type());
    EXPECT_DOUBLE_EQ(y1, old_a * x1 * x1 + old_b * x1 + old_c);
    EXPECT_DOUBLE_EQ(y1, a * x1 * x1 + b * x1 + c);
    EXPECT_DOUBLE_EQ(y2, a * x2 * x2 + b * x2 + c);
    EXPECT_DOUBLE_EQ(2 * old_a * x1 + old_b, 2 * a * x1 + b);

    std::swap(old_a, a);
    std::swap(old_b, b);
    std::swap(old_c, c);
    std::swap(x1, x2);
    std::swap(y1, y2);
    x2 = 8.5;
    y2 = 4;
    quadratic_quadrature(area, y1, x1, y2, x2, a, b, c, std::true_type());
    EXPECT_DOUBLE_EQ(y1, old_a * x1 * x1 + old_b * x1 + old_c);
    EXPECT_DOUBLE_EQ(y1, a * x1 * x1 + b * x1 + c);
    EXPECT_NEAR(y2, a * x2 * x2 + b * x2 + c, 0.001);
    EXPECT_DOUBLE_EQ(2 * old_a * x1 + old_b, 2 * a * x1 + b);
}

TEST(EulerQuadrature, simple)
{
    using namespace rbot_utils::v1;

    EulerQuadrature<double, double> q(5);
    EXPECT_DOUBLE_EQ(5, q.area);

    q(0, 0, 1, 1);
    EXPECT_DOUBLE_EQ(5.5, q.area);

    q(1, 1, 0, 2);
    EXPECT_DOUBLE_EQ(6, q.area);

    q(0, 2, 2, 3);
    EXPECT_DOUBLE_EQ(7, q.area);

    q(2, 3, 2, 4);
    EXPECT_DOUBLE_EQ(9, q.area);
}

TEST(QuadraticQuadrature, linear)
{
    using namespace rbot_utils::v1;

    QuadraticQuadrature<double, double> q(5);
    EXPECT_DOUBLE_EQ(5, q.area);

    q(0, 0, 1, 1);
    EXPECT_DOUBLE_EQ(5.5, q.area);
}

TEST(Quadrature, incomplete_tests) { EXPECT_TRUE(false); }
#endif  // _RBOT_UTILS_TEST_MATH_INTEGRATION_HPP_
