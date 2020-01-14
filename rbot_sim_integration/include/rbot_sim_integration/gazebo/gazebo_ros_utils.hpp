#ifndef _RBOT_SIM_INTEGRATION_GAZEBO_GAZEBO_ROS_UTILS_HPP_
#define _RBOT_SIM_INTEGRATION_GAZEBO_GAZEBO_ROS_UTILS_HPP_

#include <string>

#include <ros/ros.h>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {
/**
 * @fn GetRobotNamespace
 * @brief add a namespace functionality for ModelPlugins
 * @detail
 *     * see the GetRobotNamespace present in gazebo for SensorPlugin
 * @param[in] model_ model to which the plugin is added
 * @param[in] sdf_ details of the model in sdformat
 * @param[in] pInfo_ identifier for ROS_INFO
 * @return namespace unique to robot as std::string
 */
std::string GetRobotNamespace(const physics::ModelPtr &model_,
                              const sdf::ElementPtr &sdf_,
                              const char *pInfo_);
}  // namespace gazebo
#endif  // _RBOT_SIM_INTEGRATION_GAZEBO_GAZEBO_ROS_UTILS_HPP_
