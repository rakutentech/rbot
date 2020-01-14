#ifndef _RBOT_UTILS_V1_TRANSFORM_HPP_
#define _RBOT_UTILS_V1_TRANSFORM_HPP_

#include <type_traits>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rbot_utils/type_traits.hpp>

namespace rbot_utils {
namespace v1 {
using namespace Eigen;

template <class T>
constexpr Eigen::Matrix<T, 3, 1> cartesian2spherical(
    const Eigen::Matrix<T, 3, 1> &input) noexcept(false)  // @TODO
{
    Eigen::Matrix<T, 3, 1> output;

    output.x() = input.norm();
    output.y() = std::acos(input.z() / output.x());
    output.z() = std::atan2(input.y(), input.x());

    return output;
}
template <class T>
constexpr Eigen::Matrix<T, 3, 1> spherical2cartesian(
    const Eigen::Matrix<T, 3, 1> &input) noexcept(false)  // @TODO
{
    Eigen::Matrix<T, 3, 1> output;

    T intermediate = input.x() * std::sin(input.y());
    output.z() = input.x() * std::cos(input.y());

    output.x() = intermediate * std::cos(input.z());
    output.y() = intermediate * std::sin(input.z());

    return output;
}

using Transform3d = Transform<double, 3, Eigen::AffineCompact>;

static inline Quaterniond getQuaternion(double radians, const Vector3d &axis)
{
    return Quaterniond(AngleAxisd(radians, axis));
}
static inline Quaterniond getQuaternion(double radians, double x, double y, double z)
{
    return getQuaternion(radians, Vector3d(x, y, z));
}
template <class Orientation,
          class = std::enable_if_t<is_orientation<Orientation>>>
static inline Quaterniond getQuaternion(const Orientation &rot, bool normalize = true)
{
    Quaterniond q(rot.w, rot.x, rot.y, rot.z);
    if (normalize) {
        q.normalize();
    }
    return q;
}

static inline Transform3d getTransformMatrix(const Quaterniond &q,
                               const Translation3d &translation,
                               bool normalize = true)
{
    auto rot = q;
    if (normalize) {
        rot.normalize();
    }

    auto transform = translation * rot;
    return transform;
}
static inline Transform3d getTransformMatrix(const Quaterniond &q,
                               const Vector3d &pos,
                               bool normalize = true)
{
    auto t = Translation3d(pos);
    return getTransformMatrix(q, t, normalize);
}
template <class Position, class = std::enable_if_t<is_position<Position>>>
static inline Transform3d getTransformMatrix(const Quaterniond &q,
                               const Position &pos,
                               bool normalize = true)
{
    auto position = Vector3d(pos.x, pos.y, pos.z);
    return getTransformMatrix(q, position, normalize);
}
template <class Pose,
          class = std::enable_if_t<!std::is_same<Quaterniond, Pose>::value>,
          class = std::enable_if_t<is_pose<Pose>>>
static inline Transform3d getTransformMatrix(const Pose &frame, bool normalize = true)
{
    auto &orient = frame.orientation;
    Quaterniond rot(orient.w, orient.x, orient.y, orient.z);
    return getTransformMatrix(rot, frame.position, normalize);
}

namespace detail {
template <class Position, class = std::enable_if_t<is_position<Position>>>
static inline Position transformPosition(const Transform3d &world2bodyTransform,
                           const Position &bodyPoint)
{
    Vector3d pre_pos(bodyPoint.x, bodyPoint.y, bodyPoint.z);
    Vector3d post_pos = world2bodyTransform * pre_pos;

    Position worldPoint;
    worldPoint.x = post_pos[0];
    worldPoint.y = post_pos[1];
    worldPoint.z = post_pos[2];
    return worldPoint;
}
template <class Position, class = std::enable_if_t<is_position<Position>>>
static inline Position transformPositionInverse(const Transform3d &world2bodyTransform,
                                  const Position &bodyPoint)
{
    return transformPosition<Position>(world2bodyTransform.inverse(),
                                       bodyPoint);
}
}  // namespace detail

template <class Position, class = std::enable_if_t<is_position<Position>>>
static inline Position transformPosition(const Transform3d &world2bodyTransform,
                           const Position &bodyPoint)
{
    return detail::transformPosition<Position>(world2bodyTransform, bodyPoint);
}
template <class Pose,
          class Position,
          class = std::enable_if_t<!std::is_same<Pose, Transform3d>::value>,
          class = std::enable_if_t<is_pose<Pose> && is_position<Position>>>
static inline Position transformPosition(const Pose &world2bodyFrame,
                           const Position &bodyPoint,
                           bool normalize = true)
{
    return detail::transformPosition<Position>(
        getTransformMatrix(world2bodyFrame, normalize), bodyPoint);
}

template <class Position, class = std::enable_if_t<is_position<Position>>>
static inline Position transformPositionInverse(const Transform3d &world2bodyTransform,
                                  const Position &bodyPoint)
{
    return detail::transformPositionInverse<Position>(world2bodyTransform,
                                                      bodyPoint);
}
template <class Pose,
          class Position,
          class = std::enable_if_t<!std::is_same<Pose, Transform3d>::value>,
          class = std::enable_if_t<is_pose<Pose> && is_position<Position>>>
static inline Position transformPositionInverse(const Pose &world2bodyFrame,
                                  const Position &bodyPoint,
                                  bool normalize = true)
{
    return detail::transformPositionInverse<Position>(
        getTransformMatrix(world2bodyFrame, normalize), bodyPoint);
}

template <class Orientation,
          class = std::enable_if_t<is_orientation<Orientation>>>
static inline Orientation transformOrientation(const Quaterniond &prev_rotation,
                                 const Quaterniond &additional)
{
    // the order of the 2 rotations might need to be changed
    auto post_rot = prev_rotation * additional;
    Orientation rotated;
    rotated.w = post_rot.w();
    rotated.x = post_rot.x();
    rotated.y = post_rot.y();
    rotated.z = post_rot.z();
    return rotated;
}
template <class Orientation,
          class = std::enable_if_t<is_orientation<Orientation>>>
static inline Orientation transformOrientation(const Quaterniond &prev_rotation,
                                 const Orientation &additional,
                                 bool normalize = true)
{
    Quaterniond pre_rot(additional.w, additional.x, additional.y, additional.z);
    if (normalize) {
        pre_rot.normalize();
    }
    return transformOrientation<Orientation>(prev_rotation, pre_rot);
}
template <class Orientation,
          class = std::enable_if_t<is_orientation<Orientation>>>
static inline Orientation transformOrientation(const Orientation &prev_rotation,
                                 const Quaterniond &additional,
                                 bool normalize = true)
{
    Quaterniond rot(
        prev_rotation.w, prev_rotation.x, prev_rotation.y, prev_rotation.z);
    if (normalize) {
        rot.normalize();
    }
    return transformOrientation<Orientation>(rot, additional);
}
template <class Orientation,
          class = std::enable_if_t<is_orientation<Orientation>>>
static inline Orientation transformOrientation(const Orientation &prev_rotation,
                                 const Orientation &additional,
                                 bool normalize = true)
{
    Quaterniond pre_rot(additional.w, additional.x, additional.y, additional.z);
    Quaterniond rot(
        prev_rotation.w, prev_rotation.x, prev_rotation.y, prev_rotation.z);
    if (normalize) {
        pre_rot.normalize();
        rot.normalize();
    }
    return transformOrientation<Orientation>(rot, pre_rot);
}
template <
    class Pose,
    class Orientation,
    class = std::enable_if_t<!std::is_same<Pose, Orientation>::value>,
    class = std::enable_if_t<!std::is_same<Pose, Transform3d>::value>,
    class = std::enable_if_t<!std::is_same<Pose, Quaterniond>::value>,
    class = std::enable_if_t<is_pose<Pose> && is_orientation<Orientation>>>
static inline Orientation transformOrientation(const Pose &world2bodyFrame,
                                 const Orientation &bodyPoint,
                                 bool normalize = true)
{
    return transformOrientation(
        world2bodyFrame.orientation, bodyPoint, normalize);
}

template <class Orientation,
          class = std::enable_if_t<is_orientation<Orientation>>>
static inline Orientation transformOrientationInverse(const Orientation &prev_rotation,
                                        const Orientation &additional)
{
    // @TODO: use transformOrientation itself
    Quaterniond pre_rot(additional.w, additional.x, additional.y, additional.z);
    Quaterniond rot(
        prev_rotation.w, prev_rotation.x, prev_rotation.y, prev_rotation.z);
    auto post_rot = rot * pre_rot.inverse();

    Orientation rotated;
    rotated.w = post_rot.w();
    rotated.x = post_rot.x();
    rotated.y = post_rot.y();
    rotated.z = post_rot.z();
    return rotated;
}
template <
    class Pose,
    class Orientation,
    class = std::enable_if_t<!std::is_same<Pose, Orientation>::value>,
    class = std::enable_if_t<is_pose<Pose> && is_orientation<Orientation>>>
static inline Orientation transformOrientationInverse(const Pose &world2bodyFrame,
                                        const Orientation &bodyPoint)
{
    return transformOrientationInverse(world2bodyFrame.orientation, bodyPoint);
}

template <class Pose1,
          class Pose2,
          class = std::enable_if_t<!std::is_same<Pose1, Transform3d>::value>,
          class = std::enable_if_t<is_pose<Pose1> && is_pose<Pose2>>>
static inline Pose1 transformPose(const Pose1 &world2bodyFrame, const Pose2 &bodyPoint)
{
    Pose1 worldPoint;

    worldPoint.position =
        transformPosition(world2bodyFrame, bodyPoint.position);
    worldPoint.orientation =
        transformOrientation(world2bodyFrame, bodyPoint.orientation);

    return worldPoint;
}

template <class Pose, class = std::enable_if_t<is_pose<Pose>>>
static inline Pose transformPose(const Transform3d &world2bodyTransform,
                   const Pose &bodyPoint)
{
    Pose worldPoint;

    worldPoint.position =
        transformPosition(world2bodyTransform, bodyPoint.position);
    worldPoint.orientation = transformOrientation(
        Quaterniond(world2bodyTransform.rotation()), bodyPoint.orientation);

    return worldPoint;
}

template <class Pose1,
          class Pose2,
          class = std::enable_if_t<!std::is_same<Pose1, Transform3d>::value>,
          class = std::enable_if_t<is_pose<Pose1> && is_pose<Pose2>>>
static inline Pose1 transformPoseInverse(const Pose1 &world2bodyFrame,
                           const Pose2 &worldPoint)
{
    Pose1 transformed;

    transformed.position =
        transformPositionInverse(world2bodyFrame, worldPoint.position);
    transformed.orientation =
        transformOrientationInverse(world2bodyFrame, worldPoint.orientation);

    return transformed;
}

template <class Pose, class = std::enable_if_t<is_pose<Pose>>>
static inline Pose transformPoseInverse(const Transform3d &world2bodyTransform,
                          const Pose &bodyPoint)
{
    Pose worldPoint;

    worldPoint.position =
        transformPositionInverse(world2bodyTransform, bodyPoint.position);
    worldPoint.orientation =
        transformOrientationInverse(world2bodyTransform, bodyPoint.orientation);

    return worldPoint;
}
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_TRANSFORM_HPP_ */
