#ifndef _RBOT_UTILS_V1_ALGORITHM_TRANSFORM_HPP_
#define _RBOT_UTILS_V1_ALGORITHM_TRANSFORM_HPP_

namespace rbot_utils {
namespace v1 {
namespace detail {
template <class T, class U, class V, class BinaryOp>
constexpr void transform(const T &input1,
                         const U &input2,
                         V &output,
                         BinaryOp binary_op)
{
    output = binary_op(input1, input2);
}
template <class T, class U, class UnaryOp>
constexpr void transform(const T &input, U &output, UnaryOp unary_op)
{
    output = unary_op(input);
}
}  // namespace detail

template <class T, class U, class UnaryOp>
constexpr void transform_1d(const T &input, U &output, UnaryOp unary_op)
{
    detail::transform(input.x, output.x, unary_op);
}

template <class T, class U, class UnaryOp>
constexpr void transform_2d(const T &input, U &output, UnaryOp unary_op)
{
    transform_1d(input, output, unary_op);
    detail::transform(input.y, output.y, unary_op);
}

template <class T, class U, class UnaryOp>
constexpr void transform_3d(const T &input, U &output, UnaryOp unary_op)
{
    transform_2d(input, output, unary_op);
    detail::transform(input.z, output.z, unary_op);
}

template <class T, class U, class UnaryOp>
constexpr void transform_4d(const T &input, U &output, UnaryOp unary_op)
{
    transform_3d(input, output, unary_op);
    detail::transform(input.w, output.w, unary_op);
}

template <class T, class U, class UnaryOp>
constexpr void transform_pose(const T &input, U &output, UnaryOp unary_op)
{
    transform_3d(input.position, output.position, unary_op);
    transform_4d(input.orientation, output.orientation, unary_op);
}

template <class T, class U, class UnaryOp>
constexpr void transform_motion(const T &input, U &output, UnaryOp unary_op)
{
    transform_3d(input.linear, output.linear, unary_op);
    transform_3d(input.angular, output.angular, unary_op);
}

template <class T, class U, class UnaryOp>
constexpr void transform_velocity(const T &input, U &output, UnaryOp unary_op)
{
    transform_motion(input, output, unary_op);
}

template <class T, class U, class UnaryOp>
constexpr void transform_acceleration(const T &input,
                                      U &output,
                                      UnaryOp unary_op)
{
    transform_motion(input, output, unary_op);
}

template <class T, class U, class UnaryOp>
constexpr void transform_setpoint(const T &input, U &output, UnaryOp unary_op)
{
    transform_pose(input.pose, output.pose, unary_op);
    transform_velocity(input.velocity, output.velocity, unary_op);
    transform_acceleration(input.acceleration, output.acceleration, unary_op);
}

template <class T, class U, class V, class BinaryOp>
constexpr void transform_1d(const T &input1,
                            const U &input2,
                            V &output,
                            BinaryOp binary_op)
{
    detail::transform(input1.x, input2.x, output.x, binary_op);
}

template <class T, class U, class V, class BinaryOp>
constexpr void transform_2d(const T &input1,
                            const U &input2,
                            V &output,
                            BinaryOp binary_op)
{
    transform_1d(input1, input2, output, binary_op);
    detail::transform(input1.y, input2.y, output.y, binary_op);
}

template <class T, class U, class V, class BinaryOp>
constexpr void transform_3d(const T &input1,
                            const U &input2,
                            V &output,
                            BinaryOp binary_op)
{
    transform_2d(input1, input2, output, binary_op);
    detail::transform(input1.z, input2.z, output.z, binary_op);
}

template <class T, class U, class V, class BinaryOp>
constexpr void transform_4d(const T &input1,
                            const U &input2,
                            V &output,
                            BinaryOp binary_op)
{
    transform_3d(input1, input2, output, binary_op);
    detail::transform(input1.w, input2.w, output.w, binary_op);
}

template <class T, class U, class V, class BinaryOp>
constexpr void transform_pose(const T &input1,
                              const U &input2,
                              V &output,
                              BinaryOp binary_op)
{
    transform_3d(input1.position, input2.position, output.position, binary_op);
    transform_4d(
        input1.orientation, input2.orientation, output.orientation, binary_op);
}

template <class T, class U, class V, class BinaryOp>
constexpr void transform_motion(const T &input1,
                                const U &input2,
                                V &output,
                                BinaryOp binary_op)
{
    transform_3d(input1.linear, input2.linear, output.linear, binary_op);
    transform_3d(input1.angular, input2.angular, output.angular, binary_op);
}

template <class T, class U, class V, class BinaryOp>
constexpr void transform_velocity(const T &input1,
                                  const U &input2,
                                  V &output,
                                  BinaryOp binary_op)
{
    transform_motion(input1, input2, output, binary_op);
}

template <class T, class U, class V, class BinaryOp>
constexpr void transform_acceleration(const T &input1,
                                      const U &input2,
                                      V &output,
                                      BinaryOp binary_op)
{
    transform_motion(input1, input2, output, binary_op);
}

template <class T, class U, class V, class BinaryOp>
constexpr void transform_setpoint(const T &input1,
                                  const U &input2,
                                  V &output,
                                  BinaryOp binary_op)
{
    transform_pose(input1.pose, input2.pose, output.pose, binary_op);
    transform_velocity(
        input1.velocity, input2.velocity, output.velocity, binary_op);
    transform_acceleration(input1.acceleration,
                           input2.acceleration,
                           output.acceleration,
                           binary_op);
}
}  // namespace v1
}  // namespace rbot_utils
#endif  // _RBOT_UTILS_V1_ALGORITHM_TRANSFORM_HPP_
