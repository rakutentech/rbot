#ifndef _RBOT_SIM_INTEGRATION_PLUGIN_GAZEBO_ROS_RBOT_IMU_SENSOR_HPP_
#define _RBOT_SIM_INTEGRATION_PLUGIN_GAZEBO_ROS_RBOT_IMU_SENSOR_HPP_

#include <cassert>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <gazebo/sensors/ImuSensor.hh>

#include <rbot_sim_integration/gazebo/gazebo_sensor_plugin_base.hpp>
#include <rbot_sim_integration/gazebo/sdf_sensor_model.hpp>

namespace gazebo {
class GazeboRosRbotImuSensor : public GazeboSensorPluginBase
{
  public:
    GazeboRosRbotImuSensor();

    virtual ~GazeboRosRbotImuSensor() {}

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
    SensorModel3 m_accelModel;
    SensorModel3 m_gyroModel;
    SensorModel3 m_magnetoModel;

    math::Vector3 m_accel, m_gyro;
    math::Quaternion m_magneto;

    sensor_msgs::Imu m_imuMsg;
    sensor_msgs::Imu m_biasMsg;

    std::mutex m_msgMutex, m_dataMutex;

};  // class GazeboRosRbotImuSensor

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosRbotImuSensor);
}  // namespace gazebo
#endif  // _RBOT_SIM_INTEGRATION_PLUGIN_GAZEBO_ROS_RBOT_IMU_SENSOR_HPP_
