#ifndef _RBOT_SIM_INTEGRATION_PLUGIN_GAZEBO_ROS_REAL_TIME_FACTOR_HPP_
#define _RBOT_SIM_INTEGRATION_PLUGIN_GAZEBO_ROS_REAL_TIME_FACTOR_HPP_

#include <rbot_sim_integration/gazebo/gazebo_world_plugin_base.hpp>

#include <std_msgs/Float64.h>

namespace gazebo {
class GazeboRosRealTimeFactorPlugin : public GazeboWorldPluginBase
{
  public:
    GazeboRosRealTimeFactorPlugin();

  protected:
    ros::Publisher m_rtfPub;
    gazebo::transport::SubscriberPtr m_rtfSubPtr;

    gazebo::transport::NodePtr m_gzNode;

    std_msgs::Float64 m_rtfMsg;

    void m_load();

    void m_onUpdate() {}

    void m_reset() {}

    void m_spinOnce();

    void m_setupRosApi();

    void m_rtfCb(ConstDiagnosticsPtr& msg_);
};  // class GazeboRosRealTimeFactorPlugin

GZ_REGISTER_WORLD_PLUGIN(GazeboRosRealTimeFactorPlugin);
}  // namespace gazebo
#endif  // _RBOT_SIM_INTEGRATION_PLUGIN_GAZEBO_ROS_REAL_TIME_FACTOR_HPP_
