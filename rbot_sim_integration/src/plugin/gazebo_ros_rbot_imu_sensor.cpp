#include <rbot_utils/algorithm.hpp>

#include <rbot_sim_integration/plugin/gazebo_ros_rbot_imu_sensor.hpp>

namespace gazebo {
GazeboRosRbotImuSensor::GazeboRosRbotImuSensor()
    : GazeboSensorPluginBase("rbot_imu")
{
    m_reset();
}

void GazeboRosRbotImuSensor::m_load()
{
    assert(std::dynamic_pointer_cast<sensors::ImuSensor>(m_parentPtr) !=
               nullptr &&
           "Parent sensor is not ImuSensor. Plugin can't work");
    m_accelModel.Load(m_sdfPtr, "accel");
    m_gyroModel.Load(m_sdfPtr, "gyro");
    m_magnetoModel.Load(m_sdfPtr, "magneto");
}

void GazeboRosRbotImuSensor::m_onUpdate()
{
    auto imuPtr = std::dynamic_pointer_cast<sensors::ImuSensor>(m_parentPtr);

    double dt = (m_currentTime - m_prevTime).Double();

    if (dt <= 0) {
        return;
    }

    {
        std::lock_guard<std::mutex> guard(m_dataMutex);

        m_accel = imuPtr->LinearAcceleration(true);
        m_gyro = imuPtr->AngularVelocity(true);
        m_magneto = imuPtr->Orientation();

        m_accel = m_accelModel(m_accel, dt);
        m_gyro = m_gyroModel(m_gyro, dt);

        // convert x axis unit vector to final output
        auto x = math::Vector3::UnitX;
        auto vec = m_magneto.RotateVector(x);
        vec = m_magnetoModel(vec, dt);
        getQuaternion(m_magneto, x, vec);
    }

    {
        std::lock_guard<std::mutex> guard(m_msgMutex);
        rbot_utils::copy_3d(m_imuMsg.angular_velocity, m_gyro);
        rbot_utils::copy_3d(m_imuMsg.linear_acceleration, m_accel);
        rbot_utils::copy_4d(m_imuMsg.orientation, m_magneto);

        rbot_utils::copy_3d(m_biasMsg.angular_velocity,
                            m_gyroModel.getCurrentBias());
        rbot_utils::copy_3d(m_biasMsg.linear_acceleration,
                            m_accelModel.getCurrentBias());

        math::Quaternion q;
        getQuaternion(q, math::Vector3::UnitX, m_magnetoModel.getCurrentBias());
        rbot_utils::copy_4d(m_biasMsg.orientation, q);

        m_imuMsg.header.stamp = ros::Time::now();
        m_biasMsg.header = m_imuMsg.header;
    }
}

void GazeboRosRbotImuSensor::m_setupRosApi()
{
    m_dataPublisher = m_nhPtr->advertise<sensor_msgs::Imu>(m_topicName, 10);
    m_biasPublisher = m_nhPtr->advertise<sensor_msgs::Imu>(m_biasTopicName, 10);

    m_accelModel.setServices(
        std::make_unique<ros::NodeHandle>(*m_nhPtr, "accel"));
    m_gyroModel.setServices(
        std::make_unique<ros::NodeHandle>(*m_nhPtr, "gyro"));
    m_magnetoModel.setServices(
        std::make_unique<ros::NodeHandle>(*m_nhPtr, "magneto"));
}

void GazeboRosRbotImuSensor::m_spinOnce()
{
    std::lock_guard<std::mutex> guard(m_msgMutex);
    m_dataPublisher.publish(m_imuMsg);
    m_biasPublisher.publish(m_biasMsg);
}

void GazeboRosRbotImuSensor::m_reset()
{
    {
        std::lock_guard<std::mutex> guard(m_dataMutex);

        auto zero = math::Vector3::Zero;
        m_accel = zero;
        m_gyro = zero;
        m_magneto = math::Quaternion(1, 0, 0, 0);
        m_accelModel.reset(zero);
        m_gyroModel.reset(zero);
        m_magnetoModel.reset(zero);
    }

    {
        std::lock_guard<std::mutex> guard(m_msgMutex);

        m_imuMsg.header.frame_id = m_frameName;

#if GAZEBO_MAJOR_VERSION >= 9
        m_imuMsg.orientation_covariance[0] =
            std::pow(m_magnetoModel.gaussianNoise.X(), 2);
        m_imuMsg.orientation_covariance[4] =
            std::pow(m_magnetoModel.gaussianNoise.Y(), 2);
        m_imuMsg.orientation_covariance[8] =
            std::pow(m_magnetoModel.gaussianNoise.Z(), 2);
        m_imuMsg.angular_velocity_covariance[0] =
            std::pow(m_gyroModel.gaussianNoise.X(), 2);
        m_imuMsg.angular_velocity_covariance[4] =
            std::pow(m_gyroModel.gaussianNoise.Y(), 2);
        m_imuMsg.angular_velocity_covariance[8] =
            std::pow(m_gyroModel.gaussianNoise.Z(), 2);
        m_imuMsg.linear_acceleration_covariance[0] =
            std::pow(m_accelModel.gaussianNoise.X(), 2);
        m_imuMsg.linear_acceleration_covariance[4] =
            std::pow(m_accelModel.gaussianNoise.Y(), 2);
        m_imuMsg.linear_acceleration_covariance[8] =
            std::pow(m_accelModel.gaussianNoise.Z(), 2);
#else
        m_imuMsg.orientation_covariance[0] =
            std::pow(m_magnetoModel.gaussianNoise.x, 2);
        m_imuMsg.orientation_covariance[4] =
            std::pow(m_magnetoModel.gaussianNoise.y, 2);
        m_imuMsg.orientation_covariance[8] =
            std::pow(m_magnetoModel.gaussianNoise.z, 2);
        m_imuMsg.angular_velocity_covariance[0] =
            std::pow(m_gyroModel.gaussianNoise.x, 2);
        m_imuMsg.angular_velocity_covariance[4] =
            std::pow(m_gyroModel.gaussianNoise.y, 2);
        m_imuMsg.angular_velocity_covariance[8] =
            std::pow(m_gyroModel.gaussianNoise.z, 2);
        m_imuMsg.linear_acceleration_covariance[0] =
            std::pow(m_accelModel.gaussianNoise.x, 2);
        m_imuMsg.linear_acceleration_covariance[4] =
            std::pow(m_accelModel.gaussianNoise.y, 2);
        m_imuMsg.linear_acceleration_covariance[8] =
            std::pow(m_accelModel.gaussianNoise.z, 2);
#endif
    }
}
}  // namespace gazebo
