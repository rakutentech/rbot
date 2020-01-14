#ifndef _RBOT_UTILS_V1_ALGORITHM_TRANSFORM_IF_HPP_
#define _RBOT_UTILS_V1_ALGORITHM_TRANSFORM_IF_HPP_

namespace rbot_utils {
namespace v1 {
namespace detail {
template <class T, class U, class V, class BinaryOp, class W>
constexpr void transform_if(const T &input1,
                            const U &input2,
                            V &output,
                            BinaryOp binary_op,
                            const W &condition)
{
    if (condition) {
        output = binary_op(input1, input2);
    }
}
template <class T, class U, class UnaryOp, class W>
constexpr void transform_if(const T &input,
                            U &output,
                            UnaryOp unary_op,
                            const W &condition)
{
    if (condition) {
        output = unary_op(input);
    }
}
}  // namespace detail

template <class T, class U, class UnaryOp, class W>
constexpr void transform_1d_if(const T &input,
                               U &output,
                               UnaryOp unary_op,
                               const W &condition)
{
    detail::transform_if(input.x, output.x, unary_op, condition.x);
}

template <class T, class U, class UnaryOp, class W>
constexpr void transform_2d_if(const T &input,
                               U &output,
                               UnaryOp unary_op,
                               const W &condition)
{
    transform_1d_if(input, output, unary_op, condition);
    detail::transform_if(input.y, output.y, unary_op, condition.y);
}

template <class T, class U, class UnaryOp, class W>
constexpr void transform_3d_if(const T &input,
                               U &output,
                               UnaryOp unary_op,
                               const W &condition)
{
    transform_2d_if(input, output, unary_op, condition);
    detail::transform_if(input.z, output.z, unary_op, condition.y);
}

template <class T, class U, class UnaryOp, class W>
constexpr void transform_4d_if(const T &input,
                               U &output,
                               UnaryOp unary_op,
                               const W &condition)
{
    transform_3d_if(input, output, unary_op, condition);
    detail::transform_if(input.w, output.w, unary_op, condition.z);
}

template <class T, class U, class UnaryOp, class W>
constexpr void transform_pose_if(const T &input,
                                 U &output,
                                 UnaryOp unary_op,
                                 const W &condition)
{
    transform_3d_if(
        input.position, output.position, unary_op, condition.position);
    transform_4d_if(
        input.orientation, output.orientation, unary_op, condition.orientation);
}

template <class T, class U, class UnaryOp, class W>
constexpr void transform_motion_if(const T &input,
                                   U &output,
                                   UnaryOp unary_op,
                                   const W &condition)
{
    transform_3d_if(input.linear, output.linear, unary_op, condition.linear);
    transform_3d_if(input.angular, output.angular, unary_op, condition.angular);
}

template <class T, class U, class UnaryOp, class W>
constexpr void transform_velocity_if(const T &input,
                                     U &output,
                                     UnaryOp unary_op,
                                     const W &condition)
{
    transform_motion_if(input, output, unary_op, condition);
}

template <class T, class U, class UnaryOp, class W>
constexpr void transform_acceleration_if(const T &input,
                                         U &output,
                                         UnaryOp unary_op,
                                         const W &condition)
{
    transform_motion_if(input, output, unary_op, condition);
}

template <class T, class U, class UnaryOp, class W>
constexpr void transform_setpoint_if(const T &input,
                                     U &output,
                                     UnaryOp unary_op,
                                     const W &condition)
{
    transform_pose_if(input.pose, output.pose, unary_op, condition.pose);
    transform_velocity_if(
        input.velocity, output.velocity, unary_op, condition.velocity);
    transform_acceleration_if(input.acceleration,
                              output.acceleration,
                              unary_op,
                              condition.acceleration);
}

template <class T, class U, class V, class BinaryOp, class W>
constexpr void transform_1d_if(const T &input1,
                               const U &input2,
                               V &output,
                               BinaryOp binary_op,
                               const W &condition)
{
    detail::transform_if(input1.x, input2.x, output.x, binary_op, condition.x);
}

template <class T, class U, class V, class BinaryOp, class W>
constexpr void transform_2d_if(const T &input1,
                               const U &input2,
                               V &output,
                               BinaryOp binary_op,
                               const W &condition)
{
    transform_1d_if(input1, input2, output, binary_op, condition);
    detail::transform_if(input1.y, input2.y, output.y, binary_op, condition.y);
}

template <class T, class U, class V, class BinaryOp, class W>
constexpr void transform_3d_if(const T &input1,
                               const U &input2,
                               V &output,
                               BinaryOp binary_op,
                               const W &condition)
{
    transform_2d_if(input1, input2, output, binary_op, condition);
    detail::transform_if(input1.z, input2.z, output.z, binary_op, condition.z);
}

template <class T, class U, class V, class BinaryOp, class W>
constexpr void transform_4d_if(const T &input1,
                               const U &input2,
                               V &output,
                               BinaryOp binary_op,
                               const W &condition)
{
    transform_3d_if(input1, input2, output, binary_op, condition);
    detail::transform_if(input1.w, input2.w, output.w, binary_op, condition.w);
}

template <class T, class U, class V, class BinaryOp, class W>
constexpr void transform_pose_if(const T &input1,
                                 const U &input2,
                                 V &output,
                                 BinaryOp binary_op,
                                 const W &condition)
{
    transform_3d_if(input1.position,
                    input2.position,
                    output.position,
                    binary_op,
                    condition.position);
    transform_4d_if(input1.orientation,
                    input2.orientation,
                    output.orientation,
                    binary_op,
                    condition.orientation);
}

template <class T, class U, class V, class BinaryOp, class W>
constexpr void transform_motion_if(const T &input1,
                                   const U &input2,
                                   V &output,
                                   BinaryOp binary_op,
                                   const W &condition)
{
    transform_3d_if(input1.linear,
                    input2.linear,
                    output.linear,
                    binary_op,
                    condition.linear);
    transform_3d_if(input1.angular,
                    input2.angular,
                    output.angular,
                    binary_op,
                    condition.angular);
}

template <class T, class U, class V, class BinaryOp, class W>
constexpr void transform_velocity_if(const T &input1,
                                     const U &input2,
                                     V &output,
                                     BinaryOp binary_op,
                                     const W &condition)
{
    transform_motion_if(input1, input2, output, binary_op, condition);
}

template <class T, class U, class V, class BinaryOp, class W>
constexpr void transform_acceleration_if(const T &input1,
                                         const U &input2,
                                         V &output,
                                         BinaryOp binary_op,
                                         const W &condition)
{
    transform_motion_if(input1, input2, output, binary_op, condition);
}

template <class T, class U, class V, class BinaryOp, class W>
constexpr void transform_setpoint_if(const T &input1,
                                     const U &input2,
                                     V &output,
                                     BinaryOp binary_op,
                                     const W &condition)
{
    transform_pose_if(
        input1.pose, input2.pose, output.pose, binary_op, condition.pose);
    transform_velocity_if(input1.velocity,
                          input2.velocity,
                          output.velocity,
                          binary_op,
                          condition.velocity);
    transform_acceleration_if(input1.acceleration,
                              input2.acceleration,
                              output.acceleration,
                              binary_op,
                              condition.acceleration);
}
}  // namespace v1
}  // namespace rbot_utils
#endif  // _RBOT_UTILS_V1_ALGORITHM_TRANSFORM_IF_HPP_
