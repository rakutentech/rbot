#ifndef _RBOT_UTILS_V1_MATH_INTEGRATION_HPP_
#define _RBOT_UTILS_V1_MATH_INTEGRATION_HPP_

#include <cstdint>
#include <type_traits>
#include <utility>

namespace rbot_utils {
namespace v1 {
template <class T,
          class U,
          class V = decltype(std::declval<T>() * std::declval<U>())>
constexpr V prior_linear_quadrature(const T& y1,
                                           const U& x1,
                                           const T& y2,
                                           const U& x2)
{
    return (x2 - x1) * y1;
}
template <class T,
          class U,
          class V = decltype(std::declval<T>() * std::declval<U>())>
constexpr V posterior_linear_quadrature(const T& y1,
                                               const U& x1,
                                               const T& y2,
                                               const U& x2)
{
    return (x2 - x1) * y2;
}
template <class T,
          class U,
          class V = decltype(std::declval<T>() * std::declval<U>())>
constexpr V center_linear_quadrature(const T& y1,
                                            const U& x1,
                                            const T& y2,
                                            const U& x2)
{
    return (x2 - x1) * (y2 + y1) / 2;
}

template <class T,
          class U,
          class V = decltype(std::declval<T>() * std::declval<U>())>
constexpr V trapezoid_quadrature(const T& y1,
                                        const U& x1,
                                        const T& y2,
                                        const U& x2)
{
    return center_linear_quadrature(y1, x1, y2, x2);
}

template <class U, class Value, class Slope>
constexpr Value euler_quadrature(const Value& area,
                                        const U delta,
                                        const Slope& slope)
{
    Value deltaArea = delta * slope;
    area = area + deltaArea;
    return deltaArea;
}
template <class T,
          class U,
          class V = decltype(std::declval<T>() * std::declval<U>())>
constexpr V euler_quadrature(const T& y1,
                                    const U& x1,
                                    const T& y2,
                                    const U& x2)
{
    return trapezoid_quadrature(y1, x1, y2, x2);
    // technically should be
    // return euler_quadrature(0, x2 - x1, (y2 - y1) / (x2 - x1));
    // But multiplication is faster than division
}
template <class T,
          class U,
          class Value = decltype(std::declval<T>() * std::declval<U>())>
constexpr Value euler_quadrature(
    Value& area, const T& y1, const U& x1, const T& y2, const U& x2)
{
    Value deltaArea = euler_quadrature(y1, x1, y2, x2);
    area = area + deltaArea;
    return deltaArea;
}

namespace detail {
template <class U,
          class A,
          class B,
          class C,
          class Value = decltype(std::declval<C>() * std::declval<U>())>
constexpr Value quadratic_quadrature(const U& delta,
                                            const A& a,
                                            const B& b,
                                            const C& c)
{
    return (a * delta * delta * delta) / 3 + (b * delta * delta) / 2 +
           (c * delta);
}

template <class T,
          class U,
          class Value = decltype(std::declval<T>() * std::declval<U>()),
          class A = decltype(std::declval<T>() / std::declval<U>() /
                             std::declval<U>()),
          class B = decltype(std::declval<T>() / std::declval<U>()),
          class C = T>
constexpr Value quadratic_quadrature_linear(
    const T& y1, const U& x1, const T& y2, const U& x2, A& a, B& b, C& c)
{
    a = 0;
    b = (y2 - y1) / (x2 - x1);
    c = y2 - b * x2;
    return euler_quadrature(y1, x1, y2, x2);
}

template <class T,
          class U,
          class Value = decltype(std::declval<T>() * std::declval<U>()),
          class A = decltype(std::declval<T>() / std::declval<U>() /
                             std::declval<U>()),
          class B = decltype(std::declval<T>() / std::declval<U>()),
          class C = T,
          class P = T>
constexpr Value quadratic_quadrature_quadratic(Value& area,
                                                      const T& y1,
                                                      const U& x1,
                                                      const T& y2,
                                                      const U& x2,
                                                      A& a,
                                                      B& b,
                                                      C& c,
                                                      const P& p = P(0))
{
    auto delta_x = x2 - x1;
    auto denominator = delta_x * delta_x;

    c = ((y1 * x2 * x2 + y2 * x1 * x1 - 2 * y1 * x1 * x2) / delta_x -
         p * x1 * x2) /
        delta_x;
    if (x1) {
        b = 2 * (y1 - c) / x1 - p;
        a = (p - b) / (2 * x1);
    } else {
        b = p;
        a = (y2 - c) / (x2 * x2) - b / x2;
    }
    std::cout << c << "\t";
    std::cout << b << "\t";
    std::cout << a << "\t";

    auto deltaArea = quadratic_quadrature(delta_x, a, b, c);
    area = area + deltaArea;
    return deltaArea;
}
template <class T,
          class U,
          class Value = decltype(std::declval<T>() * std::declval<U>()),
          class A = decltype(std::declval<T>() / std::declval<U>() /
                             std::declval<U>()),
          class B = decltype(std::declval<T>() / std::declval<U>()),
          class C = T,
          class P = T>
constexpr Value quadratic_quadrature_quadratic(Value& area,
                                                      const T& y1,
                                                      const U& x1,
                                                      const T& y2,
                                                      const U& x2,
                                                      A& a,
                                                      B& b,
                                                      C& c,
                                                      const A& old_a,
                                                      const B& old_b)
{
    auto p = 2 * old_a * x1 + old_b;
    return quadratic_quadrature_quadratic(area, y1, x1, y2, x2, a, b, c, p);
}

template <class T,
          class U,
          class Value = decltype(std::declval<T>() * std::declval<U>()),
          class A = decltype(std::declval<T>() / std::declval<U>() /
                             std::declval<U>()),
          class B = decltype(std::declval<T>() / std::declval<U>()),
          class C = T>
constexpr Value quadratic_quadrature(
    const T& y1, const U& x1, const T& y2, const U& x2, A& a, B& b, C& c)
{
    auto delta_y = y2 - y1;
    auto delta_x = x2 - x1;
    auto denominator = delta_x * delta_x;

    a = delta_y / denominator;
    b = 2 * x1 * a;
    c = (x1 * x1 * y2 - 2 * x1 * x2 * y1 + x2 * x2 * y1) / denominator;

    return quadratic_quadrature(delta_x, a, b, c);
}

template <class T, class U, class Value, class A, class B, class C>
constexpr Value quadratic_quadrature(Value& area,
                                            const T& y1,
                                            const U& x1,
                                            const T& y2,
                                            const U& x2,
                                            A& a,
                                            B& b,
                                            C& c,
                                            std::false_type)
{
    A old_a = a;
    B old_b = b;
    C old_c = c;

    auto delta = x2 - x1;

    c = (x1 * (y2 * x1 - y1 * x2) / delta + old_c * x2) / delta;
    b = (y1 + old_c - 2 * c) / x1;
    a = (y1 - x1 * b - c) / (x1 * x1);

    auto deltaArea = quadratic_quadrature(delta, a, b, c);
    area = area + deltaArea;
    return deltaArea;
}

template <class T, class U, class Value, class A, class B, class C>
constexpr Value quadratic_quadrature(Value& area,
                                            const T& y1,
                                            const U& x1,
                                            const T& y2,
                                            const U& x2,
                                            A& a,
                                            B& b,
                                            C& c,
                                            std::true_type)
{
    A old_a = a;
    B old_b = b;
    C old_c = c;

    auto delta = x2 - x1;
    auto x1x2 = x1 * x2;

    c = ((y2 * x1 * x1 - y1 * x2 * x2) / delta + x1 * x2 * old_b +
         2 * x2 * old_c) /
        delta;
    b = old_b + 2 * (old_c - c) / x1;
    // a = (y1 - x1 * b - c) / (x1 * x1);
    a = (2 * old_a * x1 + old_b - b) / (2 * x2);

    auto deltaArea = quadratic_quadrature(delta, a, b, c);
    area = area + deltaArea;
    return deltaArea;
}
}  // namespace detail

template <class T,
          class U,
          class Value = decltype(std::declval<T>() * std::declval<U>())>
struct EulerQuadrature
{
    Value area;

    EulerQuadrature() = default;
    explicit EulerQuadrature(Value area_) : area(area_) {}

    Value operator()(const T& y1, const U& x1, const T& y2, const U& x2)
    {
        return euler_quadrature(area, y1, x1, y2, x2);
    }
};

template <class T,
          class U,
          class Value = decltype(std::declval<T>() * std::declval<U>())>
struct QuadraticQuadrature
{
  protected:
    using A =
        decltype(std::declval<T>() / std::declval<U>() / std::declval<U>());
    using B = decltype(std::declval<T>() / std::declval<U>());
    using C = T;

    uint8_t m_iterations = 0;
    A m_a;
    B m_b;
    C m_c;

  public:
    Value area;

    QuadraticQuadrature() = default;
    explicit QuadraticQuadrature(Value area_) : area(area_) {}

    QuadraticQuadrature(Value area_, A a, B b, C c)
        : area(area_), m_a(a), m_b(b), m_c(c)
    {}

    void initArea(Value area_)
    {
        area = area_;
        m_iterations = 0;
    }

    Value operator()(const T& y1, const U& x1, const T& y2, const U& x2)
    {
        Value deltaArea;
        switch (m_iterations) {
            case 0:
                std::cout << "Case 0\n";
                deltaArea = detail::quadratic_quadrature_quadratic(
                    area, y1, x1, y2, x2, m_a, m_b, m_c, 0);
                m_iterations++;
                break;
            case 1:
                std::cout << "Case 1\n";
                deltaArea = detail::quadratic_quadrature_quadratic(
                    area, y1, x1, y2, x2, m_a, m_b, m_c, m_a, m_b);
                break;
            case 2:
                deltaArea = detail::quadratic_quadrature(
                    area, y1, x1, y2, x2, m_a, m_b, m_c, std::true_type());
                break;
        }
        area = area + deltaArea;
        return deltaArea;
    }
};
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_MATH_INTEGRATION_HPP_ */
