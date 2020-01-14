#ifndef _RBOT_SIM_INTEGRATION_GAZEBO_GAZEBO_UTILS_HPP_
#define _RBOT_SIM_INTEGRATION_GAZEBO_GAZEBO_UTILS_HPP_

#include <cctype>
#include <exception>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#if GAZEBO_MAJOR_VERSION >= 9
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#else
#include <gazebo/math/Quaternion.hh>
#include <gazebo/math/Vector3.hh>
#endif  // if GAZEBO_MAJOR_VERSION >= 9

namespace gazebo {
#if GAZEBO_MAJOR_VERSION >= 9
// use ignition because gazebo has no math of its own
using namespace ignition;

namespace math {
using Vector3 = ::ignition::math::Vector3d;
using Quaternion = ::ignition::math::Quaterniond;
}  // namespace math
#endif  // if GAZEBO_MAJOR_VERSION >= 9

/**
 * @fn setFromSdf
 * @template T type of value to be extracted (I've used std::string, double)
 * @brief safely read a variable value from sdf
 * @param[in] sdfPtr_ sdf pointer to read from
 * @param[in] element_ name-tag to read data from
 * @param[out] var_ variable to store data in
 * @param[in] default_ default value, in case there is no element_ in sdfPtr_
 * @return
 *     * 0 for successful find,
 *     * 1 for using default_
 *     * -1 for sdfPtr_ == nullptr
 */
template <class T>
int setFromSdf(const sdf::ElementPtr sdfPtr_,
               const std::string element_,
               T &var_,
               const T &default_)
{
    if (!sdfPtr_) {
        return -1;
    }
    if (sdfPtr_->HasElement(element_)) {
        var_ = sdfPtr_->Get<T>(element_);
        return 0;
    }
    var_ = default_;
    return 1;
}

/**
 * @fn validElementName
 * @brief check if the string is a valid SDF element name prefix
 * @detail a string is valid if it
 *     * begins with alphabet
 *     * is alphanumeric or contains hyphen, underscore or period only
 * @param[in] str_ string to check
 * @return truthiness
 */
bool isValidElementName(const std::string str_);

/**
 * @class GetQuaternionNotImplemented
 * @brief Exception to throw if getQuaternion is not specialised for that
 * override
 */
struct GetQuaternionNotImplemented : public std::bad_function_call
{
    const char *what() const noexcept
    {
        return "Can't create quaternions. Function not implemented";
    }
};

/**
 * @fn getQuaternion
 * @template T type of quaternoin
 * @template U type of vectors
 * @brief returns a quaternion required to convert one vector to another
 * @detail obv the magnitude of the vectors does not matter
 *         strongly recommended to normalize them before hand
 * @paran[out] quat_ required quaternion
 * @param[in] initial_ initial vector
 * @param[in] final_ the output quaternion will rotate inital to this vector
 */
template <typename T, typename U>
void getQuaternion(T & /*quat_*/, const U & /*initial_*/, const U & /*final_*/)
{
    throw GetQuaternionNotImplemented();
    // can't implement without dot, cross, sq len, normalise functions
    // q.xyz = a cross b
    // q.w = sqrt(a.len**2 * b.len**2) + a dot b
    // now normalise q
}

template <>
void getQuaternion<math::Quaternion, math::Vector3>(
    math::Quaternion &quat_,
    const math::Vector3 &initial_,
    const math::Vector3 &final_);
}  // namespace gazebo
#endif  // _RBOT_SIM_INTEGRATION_GAZEBO_GAZEBO_UTILS_HPP_
