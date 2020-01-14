#include <rbot_utils/algorithm.hpp>

#include <rbot_sim_integration/gazebo/gazebo_utils.hpp>

namespace gazebo {
bool isValidElementName(const std::string str_)
{
    if (str_.empty()) {
        return false;
    }
    if (!isalpha(str_[0])) {
        return false;
    }
    std::string symbols("-_.");
    auto invalidStr = [&](const char ch) {
        return !isalnum(ch) && (symbols.find(ch) == std::string::npos);
    };
    return (str_.end() == std::find_if(str_.begin(), str_.end(), invalidStr));
}

template <>
void getQuaternion<math::Quaternion, math::Vector3>(
    math::Quaternion &quat_,
    const math::Vector3 &initial_,
    const math::Vector3 &final_)
{
    auto rot = initial_.Cross(final_);
#if GAZEBO_MAJOR_VERSION >= 9
    quat_.X() = rot.X();
    quat_.W() = std::sqrt(initial_.SquaredLength() * final_.SquaredLength()) +
                initial_.Dot(final_);
#else
    rbot_utils::copy_3d(quat_, rot);
    quat_.w =
        std::sqrt(initial_.GetSquaredLength() * final_.GetSquaredLength()) +
        initial_.Dot(final_);
#endif
    quat_.Normalize();
}
}  // namespace gazebo
