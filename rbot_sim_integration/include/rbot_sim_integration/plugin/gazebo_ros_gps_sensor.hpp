#ifndef _RBOT_SIM_INTEGRATION_PLUGIN_GAZEBO_ROS_GPS_SENSOR_HPP_
#define _RBOT_SIM_INTEGRATION_PLUGIN_GAZEBO_ROS_GPS_SENSOR_HPP_

#include <cassert>
#include <memory>
#include <mutex>

#include <ros/ros.h>

#include <gazebo/common/common.hh>
#include <gazebo/sensors/GpsSensor.hh>

#include <rbot_msgs/NavSatFix.h>

#include <rbot_sim_integration/gazebo/gazebo_sensor_plugin_base.hpp>
#include <rbot_sim_integration/gazebo/sdf_sensor_model.hpp>

namespace gazebo {
class GazeboRosGpsSensor : public GazeboSensorPluginBase
{
  public:
    GazeboRosGpsSensor();

    virtual ~GazeboRosGpsSensor() {}

  protected:
    using SensorModel3 =
        rbot_sim_integration::RosSdfSensorModel<gazebo::math::Vector3>;
    using SensorModel = rbot_sim_integration::RosSdfSensorModel<double>;

    void m_load();

    void m_onUpdate();

    void m_setupRosApi();

    void m_spinOnce();

    void m_reset();

    // neglect temperature for now
    SensorModel3 m_posModel;
    SensorModel3 m_velModel;

    math::Vector3 m_pos, m_vel;

    // dilution of precision
    math::Vector3 m_dop;

    rbot_msgs::NavSatFix m_gpsMsg;
    rbot_msgs::NavSatFix m_biasMsg;

    std::mutex m_msgMutex, m_dataMutex;

};  // class GazeboRosGpsSensor

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosGpsSensor);
}  // namespace gazebo
#endif  // _RBOT_SIM_INTEGRATION_PLUGIN_GAZEBO_ROS_GPS_SENSOR_HPP_
