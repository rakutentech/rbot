#ifndef _RBOT_SIM_INTEGRATION_PLUGIN_GAZEBO_ROS_TF_HPP_
#define _RBOT_SIM_INTEGRATION_PLUGIN_GAZEBO_ROS_TF_HPP_

#include <string>
#include <thread>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <rbot_sim_integration/gazebo/gazebo_model_plugin_base.hpp>

namespace gazebo {
/**
 * @class GazeboRosTf
 * @brief Plugin to publish a Transform of model's world position and
 * orientation
 * @detail
 *     * set <publish>false</publish> to stop publishing
 *     * set <childFrame>map2</childFrame> to modify the child frame
 *     * header frame is set to map
 *     * child frame is set to modelName/childFrame
 */
class GazeboRosTf : public GazeboModelPluginBase
{
  public:
    GazeboRosTf();

    virtual ~GazeboRosTf() {}

  private:
    /**
     * @fn m_load
     * @brief loads publish and childFrame
     */
    void m_load();

    /**
     * @fn m_onUpdate
     * @brief calls m_prepareTf
     */
    void m_onUpdate();

    void m_reset() {}

    /**
     * @fn m_spinOnce
     * @brief broadcasts the prepared transform
     */
    void m_spinOnce();

    void m_setupRosApi() {}

    /**
     * @fn m_prepareTf
     * @brief saves the model's world pose data from gazebo if publish flag is
     * set true
     */
    void m_prepareTf();

    bool m_publish = true;

    std::string m_childFrame = "map";

    tf::TransformBroadcaster m_tfBroadcaster;

    ///< transform is prepared and saved in m_odom
    geometry_msgs::TransformStamped m_odom;
};  // class GazeboRosTf

GZ_REGISTER_MODEL_PLUGIN(GazeboRosTf);
}  // namespace gazebo

#endif  // _RBOT_SIM_INTEGRATION_PLUGIN_GAZEBO_ROS_TF_HPP_
