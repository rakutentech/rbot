#ifndef _RBOT_SIM_INTEGRATION_GAZEBO_GAZEBO_SENSOR_PLUGIN_BASE_HPP_
#define _RBOT_SIM_INTEGRATION_GAZEBO_GAZEBO_SENSOR_PLUGIN_BASE_HPP_

#include <memory>
#include <string>
#include <thread>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_plugins/gazebo_ros_utils.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <rbot_sim_integration/gazebo/gazebo_ros_utils.hpp>
#include <rbot_sim_integration/gazebo/gazebo_utils.hpp>

namespace gazebo {
/**
 * @class GazeboSensorPluginBase
 * @brief Does a lot of boilerplate work for sensor plugins
 */
class GazeboSensorPluginBase : public SensorPlugin
{
  public:
    /**
     * @ctor
     * @param[in] pluginName_ name of the plugin, to clarify in error messages
     */
    GazeboSensorPluginBase(const std::string pluginName_);

    /**
     * @dtor
     * @brief ensures proper closure of threads
     * @detail
     *     * virtual because inheritance
     */
    virtual ~GazeboSensorPluginBase();

    /**
     * @fn Load
     * @brief called to load the plugin. Entrypoint
     * @detail
     *     * starts 2 threads (m_loadThread, m_rosSpinThread)
     *     * registers a world update event to OnUpdate
     * @param[in] parent_ base sensor for the plugin
     * @param[in] sdf_ data for the plugin in sdformat
     */
    void Load(sensors::SensorPtr parent_, sdf::ElementPtr sdf_) override;

    /**
     * @fn OnUpdate
     * @brief entrypoint on world update, each simulation step
     * @detail delegates call to m_onUpdate
     */
    void OnUpdate();

    /**
     * @fn Reset
     * @brief called on model reset
     */
    void Reset() override;

  protected:
    /**
     * @fn m_load
     * @brief called at the end of Load, before m_setupRosApi
     */
    virtual void m_load() = 0;

    /**
     * @fn m_onUpdate
     * @brief called at each time step
     */
    virtual void m_onUpdate() = 0;

    /**
     * @fn m_reset()
     * @brief called on model reset
     */
    virtual void m_reset() = 0;

    /**
     * @fn m_spinOnce
     * @brief called once every ros spin.
     * @detail
     *     * Call all publishers and subscribers here
     */
    virtual void m_spinOnce() = 0;

    /**
     * @fn m_setupRosApi
     * @brief setup the ROS pub, sub etc here
     * @detail
     *     * All data should be already loaded via m_load
     */
    virtual void m_setupRosApi() = 0;

    /**
     * @fn m_calibrateCb
     * @brief callback for service call to calibrate sensor
     */
    virtual bool m_calibrateCb(std_srvs::Empty::Request &req_,
                               std_srvs::Empty::Response &res_);

    ///< parent sensor of the plugin
    sensors::SensorPtr m_parentPtr;

    ///< plugin information stored as sdformat data
    sdf::ElementPtr m_sdfPtr;

    ///< current and previous time stamps (for any calculation, updated in
    ///< OnUpdate)
    common::Time m_currentTime, m_prevTime;

    ///< event connection for world update
    event::ConnectionPtr m_updateConnectionPtr;

    std::string m_pluginName;
    std::string m_sensorName;
    std::string m_frameName;
    std::string m_topicName, m_biasTopicName, m_serviceName;

    std::string m_robotNamespace = "";

    std::unique_ptr<ros::NodeHandle> m_nhPtr;

    ros::Publisher m_dataPublisher;
    ros::Publisher m_biasPublisher;
    ros::ServiceServer m_calibrateSrv;

    ///< ros spin rate
    double m_rate = 0;

  private:
    /**
     * @fn m_loadBasic
     * @brief loads the basic boiler plate
     * @detail
     *     * threaded to make loading plugins faster
     *     * calls m_load and m_setupRosApi (in this order)
     */
    void m_loadBasic();

    /**
     * @fn m_spin
     * @brief ros spin thread
     * @detail
     *     * waits for m_loadBasic to get over first
     *     * calls m_spinOnce every time in loop
     *     * maintains a soft rate of m_rate via sleeping
     */
    void m_spin();

    std::thread m_rosSpinThread, m_loadThread;
};  // class GazeboSensorPluginBase
}  // namespace gazebo

#endif  // _RBOT_SIM_INTEGRATION_GAZEBO_GAZEBO_SENSOR_PLUGIN_BASE_HPP_
