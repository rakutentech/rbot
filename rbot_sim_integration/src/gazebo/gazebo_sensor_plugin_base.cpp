#include <rbot_sim_integration/gazebo/gazebo_sensor_plugin_base.hpp>

#include <sstream>

namespace gazebo {
GazeboSensorPluginBase::GazeboSensorPluginBase(std::string pluginName_)
    : m_pluginName(pluginName_)
{}

GazeboSensorPluginBase::~GazeboSensorPluginBase() { m_rosSpinThread.join(); }

void GazeboSensorPluginBase::Load(sensors::SensorPtr parent_,
                                  sdf::ElementPtr sdf_)
{
    if (!parent_) {
        gzerr << "Invalid sensor pointer\n";
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
        std::bind(&GazeboSensorPluginBase::OnUpdate, this));
    m_loadThread = std::thread(&GazeboSensorPluginBase::m_loadBasic, this);
    m_rosSpinThread = std::thread(&GazeboSensorPluginBase::m_spin, this);
}

void GazeboSensorPluginBase::OnUpdate()
{
    m_prevTime = m_currentTime;
    m_currentTime = common::Time::GetWallTime();
    return m_onUpdate();
}

void GazeboSensorPluginBase::Reset() { m_reset(); }

void GazeboSensorPluginBase::m_loadBasic()
{
    m_sensorName = m_parentPtr->Name();

    setFromSdf(m_sdfPtr, "frameName", m_frameName, m_sensorName);
    ROS_INFO("[%s] Gazebo-ROS Plugin: Set frame id as %s",
             m_sensorName.c_str(),
             m_frameName.c_str());

    setFromSdf(m_sdfPtr, "topicName", m_topicName, m_sensorName);
    ROS_INFO("[%s] Gazebo-ROS Plugin: Set topic as %s",
             m_sensorName.c_str(),
             m_topicName.c_str());

    setFromSdf(
        m_sdfPtr, "biasTopicName", m_biasTopicName, m_topicName + "/bias");
    ROS_INFO("[%s] Gazebo-ROS Plugin: Set bias topic as %s",
             m_sensorName.c_str(),
             m_biasTopicName.c_str());

    setFromSdf(
        m_sdfPtr, "serviceName", m_serviceName, m_topicName + "/calibrate");
    ROS_INFO("[%s] Gazebo-ROS Plugin: Set service as %s",
             m_sensorName.c_str(),
             m_serviceName.c_str());

    setFromSdf(m_sdfPtr, "updateRate", m_rate, 10.);
    ROS_INFO("[%s] Gazebo-ROS Plugin: Set update rate as %f",
             m_sensorName.c_str(),
             m_rate);

    m_robotNamespace =
        GetRobotNamespace(m_parentPtr, m_sdfPtr, m_sensorName.c_str());
    m_nhPtr = std::make_unique<ros::NodeHandle>(m_robotNamespace);

    gzdbg << "Gazebo-ROS Plugin: Setting up for node: " << m_sensorName << "\n";
    m_load();

    m_calibrateSrv = m_nhPtr->advertiseService(
        m_serviceName, &GazeboSensorPluginBase::m_calibrateCb, this);

    m_setupRosApi();
    ROS_INFO("[%s] Gazebo-ROS plugin %s is ready",
             m_sensorName.c_str(),
             m_pluginName.c_str());
}

void GazeboSensorPluginBase::m_spin()
{
    m_loadThread.join();
    ros::Rate rate(m_rate);
    while (ros::ok()) {
        ros::spinOnce();

        m_spinOnce();

        rate.sleep();
    }
}

bool GazeboSensorPluginBase::m_calibrateCb(std_srvs::Empty::Request &,
                                           std_srvs::Empty::Response &)
{
    Reset();
    return true;
}
}  // namespace gazebo
