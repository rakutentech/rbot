#include <rbot_sim_integration/gazebo/gazebo_model_plugin_base.hpp>

#include <sstream>

namespace gazebo {
GazeboModelPluginBase::GazeboModelPluginBase(std::string pluginName_)
    : m_pluginName(pluginName_)
{}

GazeboModelPluginBase::~GazeboModelPluginBase() { m_rosSpinThread.join(); }

void GazeboModelPluginBase::Load(physics::ModelPtr parent_,
                                 sdf::ElementPtr sdf_)
{
    if (!parent_) {
        gzerr << "Invalid model pointer\n";
        return;
    }
    m_parentPtr = parent_;
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
        std::bind(&GazeboModelPluginBase::OnUpdate, this));
    m_loadThread = std::thread(&GazeboModelPluginBase::m_loadBasic, this);
    m_rosSpinThread = std::thread(&GazeboModelPluginBase::m_spin, this);
}

void GazeboModelPluginBase::OnUpdate()
{
    m_prevTime = m_currentTime;
    m_currentTime = common::Time::GetWallTime();
    return m_onUpdate();
}

void GazeboModelPluginBase::Reset() { m_reset(); }

void GazeboModelPluginBase::m_loadBasic()
{
    m_modelName = m_sdfPtr->GetParent()->Get<std::string>("name");

    m_worldPtr = m_parentPtr->GetWorld();

    setFromSdf(m_sdfPtr, "updateRate", m_rate, 10.);

    ROS_INFO("[%s] Gazebo-ROS Plugin: Set update rate as %f",
             m_modelName.c_str(),
             m_rate);

    m_robotNamespace =
        GetRobotNamespace(m_parentPtr, m_sdfPtr, m_modelName.c_str());

    m_nhPtr = std::make_unique<ros::NodeHandle>(m_robotNamespace);

    gzdbg << "Gazebo-ROS Plugin: Setting up for node: " << m_modelName << "\n";
    m_load();

    m_setupRosApi();
    ROS_INFO("[%s] Gazebo-ROS plugin %s is ready",
             m_modelName.c_str(),
             m_pluginName.c_str());
}

void GazeboModelPluginBase::m_spin()
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
