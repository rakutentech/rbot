#include <rbot_utils/algorithm.hpp>

#include <rbot_sim_integration/plugin/gazebo_ros_gps_sensor.hpp>

namespace gazebo {
GazeboRosGpsSensor::GazeboRosGpsSensor() : GazeboSensorPluginBase("rbot_gps")
{
    m_reset();
}

void GazeboRosGpsSensor::m_load()
{
    assert(std::dynamic_pointer_cast<sensors::GpsSensor>(m_parentPtr) !=
               nullptr &&
           "Parent sensor is not GpsSensor. Plugin can't work");
    setFromSdf(m_sdfPtr, "DoP", m_dop, m_dop);
    m_posModel.Load(m_sdfPtr, "pos");
    m_velModel.Load(m_sdfPtr, "vel");
}

void GazeboRosGpsSensor::m_onUpdate()
{
    auto gpsPtr = std::dynamic_pointer_cast<sensors::GpsSensor>(m_parentPtr);

    double dt = (m_currentTime - m_prevTime).Double();

    if (dt <= 0) {
        return;
    }

    {
        std::lock_guard<std::mutex> guard(m_dataMutex);

        // this needs spherical error
        common::SphericalCoordinates sphCoord;
        ignition::math::Vector3d pos;
        pos.X(gpsPtr->Longitude().Degree());
        pos.Y(gpsPtr->Latitude().Degree());
        pos.Z(gpsPtr->Altitude());

        auto xyz = sphCoord.LocalFromSpherical(pos);
        rbot_utils::copy_3d(pos, m_posModel(xyz, dt));
        m_pos = sphCoord.SphericalFromLocal(pos);

        // @TODO: same for velocity after gazebo PR gets released
    }

    {
        std::lock_guard<std::mutex> guard(m_msgMutex);

        m_gpsMsg.header.stamp = ros::Time::now();
#if GAZEBO_MAJOR_VERSION >= 9
        m_gpsMsg.latitude = m_pos.X();
        m_gpsMsg.longitude = m_pos.Y();
        m_gpsMsg.altitude = m_pos.Z();
        m_gpsMsg.velocity.north = m_vel.X();
        m_gpsMsg.velocity.east = m_vel.Y();
        m_gpsMsg.velocity.down = -m_vel.Z();
#else
        m_gpsMsg.latitude = m_pos.x;
        m_gpsMsg.longitude = m_pos.y;
        m_gpsMsg.altitude = m_pos.z;
        m_gpsMsg.velocity.north = m_vel.x;
        m_gpsMsg.velocity.east = m_vel.y;
        m_gpsMsg.velocity.down = -m_vel.z;
#endif

        m_biasMsg.header = m_gpsMsg.header;
        auto posError = m_posModel.getCurrentBias();
#if GAZEBO_MAJOR_VERSION >= 9
        m_gpsMsg.latitude = posError.X();
        m_gpsMsg.longitude = posError.Y();
        m_gpsMsg.altitude = posError.Z();
#else
        m_gpsMsg.latitude = posError.x;
        m_gpsMsg.longitude = posError.y;
        m_gpsMsg.altitude = posError.z;
#endif
        auto velError = m_velModel.getCurrentBias();
#if GAZEBO_MAJOR_VERSION >= 9
        m_gpsMsg.velocity.north = velError.X();
        m_gpsMsg.velocity.east = velError.Y();
        m_gpsMsg.velocity.down = -velError.Z();
#else
        m_gpsMsg.velocity.north = velError.x;
        m_gpsMsg.velocity.east = velError.y;
        m_gpsMsg.velocity.down = -velError.z;
#endif
    }
}

void GazeboRosGpsSensor::m_setupRosApi()
{
    m_dataPublisher = m_nhPtr->advertise<rbot_msgs::NavSatFix>(m_topicName, 10);
    m_biasPublisher =
        m_nhPtr->advertise<rbot_msgs::NavSatFix>(m_biasTopicName, 10);

    m_posModel.setServices(std::make_unique<ros::NodeHandle>(*m_nhPtr, "pos"));
    m_velModel.setServices(std::make_unique<ros::NodeHandle>(*m_nhPtr, "vel"));
}

void GazeboRosGpsSensor::m_spinOnce()
{
    std::lock_guard<std::mutex> guard(m_msgMutex);
    m_dataPublisher.publish(m_gpsMsg);
    m_biasPublisher.publish(m_biasMsg);
}

void GazeboRosGpsSensor::m_reset()
{
    {
        std::lock_guard<std::mutex> guard(m_dataMutex);

        auto zero = math::Vector3::Zero;
        m_pos = zero;
        m_vel = zero;
        m_dop = zero;
        m_posModel.reset(zero);
        m_velModel.reset(zero);
    }

    {
        std::lock_guard<std::mutex> guard(m_msgMutex);

        m_gpsMsg.header.frame_id = m_frameName;

        m_gpsMsg.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        m_gpsMsg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS |
                                  sensor_msgs::NavSatStatus::SERVICE_GLONASS;

        m_gpsMsg.position_covariance_type =
            rbot_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        m_gpsMsg.velocity_covariance_type =
            rbot_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

#if GAZEBO_MAJOR_VERSION >= 9
        m_gpsMsg.position_covariance[0] =
            std::pow(m_posModel.gaussianNoise.X(), 2);
        m_gpsMsg.position_covariance[0] += std::pow(m_dop.X(), 2);
        m_gpsMsg.position_covariance[4] =
            std::pow(m_posModel.gaussianNoise.Y(), 2);
        m_gpsMsg.position_covariance[4] += std::pow(m_dop.Y(), 2);
        m_gpsMsg.position_covariance[8] =
            std::pow(m_posModel.gaussianNoise.Z(), 2);
        m_gpsMsg.position_covariance[8] += std::pow(m_dop.Z(), 2);

        m_gpsMsg.velocity_covariance[0] =
            std::pow(m_velModel.gaussianNoise.X(), 2);
        m_gpsMsg.velocity_covariance[4] =
            std::pow(m_velModel.gaussianNoise.Y(), 2);
        m_gpsMsg.velocity_covariance[8] =
            std::pow(m_velModel.gaussianNoise.Z(), 2);
#else
        m_gpsMsg.position_covariance[0] =
            std::pow(m_posModel.gaussianNoise.x, 2);
        m_gpsMsg.position_covariance[0] += std::pow(m_dop.x, 2);
        m_gpsMsg.position_covariance[4] =
            std::pow(m_posModel.gaussianNoise.y, 2);
        m_gpsMsg.position_covariance[4] += std::pow(m_dop.y, 2);
        m_gpsMsg.position_covariance[8] =
            std::pow(m_posModel.gaussianNoise.z, 2);
        m_gpsMsg.position_covariance[8] += std::pow(m_dop.z, 2);

        m_gpsMsg.velocity_covariance[0] =
            std::pow(m_velModel.gaussianNoise.x, 2);
        m_gpsMsg.velocity_covariance[4] =
            std::pow(m_velModel.gaussianNoise.y, 2);
        m_gpsMsg.velocity_covariance[8] =
            std::pow(m_velModel.gaussianNoise.z, 2);
#endif
    }
}
}  // namespace gazebo
