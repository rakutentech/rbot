#include <rbot_sim_integration/gazebo/gazebo_world_plugin_base.hpp>

#include <sstream>

namespace gazebo {
GazeboWorldPluginBase::GazeboWorldPluginBase(std::string pluginName_)
    : m_pluginName(pluginName_)
{}

GazeboWorldPluginBase::~GazeboWorldPluginBase() { m_rosSpinThread.join(); }

void GazeboWorldPluginBase::Load(physics::WorldPtr parent_,
                                 sdf::ElementPtr sdf_)
{
    if (!parent_) {
        gzerr << "Invalid world pointer\n";
        return;
    }
    m_worldPtr = parent_;
    if (!sdf_) {
        gzerr << "Invalid sdf pointer\n";
        return;
    }
    m_sdfPtr = sdf_;

    if (!ros::isInitialized()) {
        ROS_FATAL(
            "A ROS node for Gazebo has not been initialized, unable to load plugin. \
                   Load the Gazebo system pugin '%s' in the gazebo_ros package",
            m_pluginName.c_str());
        return;
    }

    m_updateConnectionPtr = event::Events::ConnectWorldUpdateEnd(
        std::bind(&GazeboWorldPluginBase::OnUpdate, this));
    m_loadThread = std::thread(&GazeboWorldPluginBase::m_loadBasic, this);
    m_rosSpinThread = std::thread(&GazeboWorldPluginBase::m_spin, this);
}

void GazeboWorldPluginBase::OnUpdate()
{
    m_prevTime = m_currentTime;
    m_currentTime = common::Time::GetWallTime();
    return m_onUpdate();
}

void GazeboWorldPluginBase::Reset() { m_reset(); }

void GazeboWorldPluginBase::m_loadBasic()
{
    m_worldName = m_sdfPtr->Get<std::string>("name");

    setFromSdf(m_sdfPtr, "updateRate", m_rate, 10.);

    ROS_INFO("[%s] Gazebo-ROS Plugin: Set update rate as %f",
             m_worldName.c_str(),
             m_rate);

    // No need for this?
    // m_robotNamespace = GetRobotNamespace(m_worldPtr, m_sdfPtr,
    // m_worldName.c_str());
    m_robotNamespace = "";

    m_nhPtr = std::make_unique<ros::NodeHandle>(m_robotNamespace);

    gzdbg << "Gazebo-ROS Plugin: Setting up for node: " << m_worldName << "\n";
    m_load();

    m_setupRosApi();
    ROS_INFO("[%s] Gazebo-ROS plugin %s is ready",
             m_worldName.c_str(),
             m_pluginName.c_str());
}

void GazeboWorldPluginBase::m_spin()
{
    m_loadThread.join();
    ros::Rate rate(m_rate);
    while (ros::ok()) {
        ros::spinOnce();

        m_spinOnce();

        rate.sleep();
    }
}
}  // namespace gazebo
