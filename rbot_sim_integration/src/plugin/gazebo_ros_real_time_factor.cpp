#include <rbot_sim_integration/plugin/gazebo_ros_real_time_factor.hpp>

namespace gazebo {
GazeboRosRealTimeFactorPlugin::GazeboRosRealTimeFactorPlugin()
    : GazeboWorldPluginBase("rbot_real_time_factor")
{}

void GazeboRosRealTimeFactorPlugin::m_load()
{
    m_gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
    m_gzNode->Init();

    m_rtfSubPtr = m_gzNode->Subscribe("/gazebo/default/diagnostics",
                                      &GazeboRosRealTimeFactorPlugin::m_rtfCb,
                                      this);
}

void GazeboRosRealTimeFactorPlugin::m_spinOnce() { m_rtfPub.publish(m_rtfMsg); }

void GazeboRosRealTimeFactorPlugin::m_setupRosApi()
{
    m_rtfPub =
        m_nhPtr->advertise<std_msgs::Float64>("gazebo/real_time_factor", 1);
}

void GazeboRosRealTimeFactorPlugin::m_rtfCb(ConstDiagnosticsPtr& msg_)
{
    m_rtfMsg.data = msg_->real_time_factor();
}
}  // namespace gazebo
