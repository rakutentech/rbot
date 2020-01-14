#ifndef _RBOT_SIM_INTEGRATION_GAZEBO_SDF_SENSOR_MODEL_HPP_
#define _RBOT_SIM_INTEGRATION_GAZEBO_SDF_SENSOR_MODEL_HPP_

#include <ros/ros.h>

#include <sdf/sdf.hh>

#include <rbot_msgs/CalibrateSensorModel1D.h>
#include <rbot_msgs/CalibrateSensorModel3D.h>

#include <rbot_utils/algorithm.hpp>

#include <rbot_sim_integration/gazebo/gazebo_utils.hpp>
#include <rbot_sim_integration/gazebo/sensor_model.hpp>

namespace rbot_sim_integration {
template <typename T>
struct SdfSensorModel : public SensorModel<T>
{
    using SensorModel<T>::SensorModel;

    virtual void Load(sdf::ElementPtr sdfPtr_, const std::string &prefix_ = "")
    {
        std::string offset("offset"), drift("drift"), scaleError("scaleError");
        std::string driftFreq("driftFrequency"), gaussianNoise("gaussianNoise");
        if (gazebo::isValidElementName(prefix_)) {
            offset[0] = toupper(offset[0]);
            drift[0] = toupper(drift[0]);
            scaleError[0] = toupper(scaleError[0]);
            driftFreq[0] = toupper(driftFreq[0]);
            gaussianNoise[0] = toupper(gaussianNoise[0]);

            offset = prefix_ + offset;
            drift = prefix_ + drift;
            scaleError = prefix_ + scaleError;
            driftFreq = prefix_ + driftFreq;
            gaussianNoise = prefix_ + gaussianNoise;
        }

        T defaultVal;
        defaultVal = 0;
        gazebo::setFromSdf(sdfPtr_, offset, this->offset, defaultVal);
        gzdbg << "Set " << offset << " as " << this->offset << "\n";
        gazebo::setFromSdf(sdfPtr_, drift, this->drift, defaultVal);
        gzdbg << "Set " << drift << " as " << this->drift << "\n";
        gazebo::setFromSdf(
            sdfPtr_, gaussianNoise, this->gaussianNoise, defaultVal);
        gzdbg << "Set " << gaussianNoise << " as " << this->gaussianNoise
              << "\n";

        gazebo::setFromSdf(
            sdfPtr_, driftFreq, this->driftFreq, this->driftFreq);
        gzdbg << "Set " << driftFreq << " as " << this->driftFreq << "\n";
        gazebo::setFromSdf(sdfPtr_, scaleError, this->scale, this->scale);
        gzdbg << "Set " << scaleError << " as " << this->scale << "\n";
    }
};  // class SdfSensorModel

template <typename T>
class RosSdfSensorModel : public SdfSensorModel<T>
{
  public:
    using SdfSensorModel<T>::SdfSensorModel;
    void setServices(std::unique_ptr<ros::NodeHandle> nhPtr_);

  protected:
    ros::ServiceServer m_calibrateSrv;
};  // class RosSdfSensorModel

template <>
class RosSdfSensorModel<double> : public SdfSensorModel<double>
{
  public:
    using SdfSensorModel<double>::SdfSensorModel;
    void setServices(std::unique_ptr<ros::NodeHandle> nhPtr_)
    {
        m_calibrateSrv =
            nhPtr_->advertiseService("sensor_model/calibrate",
                                     &RosSdfSensorModel<double>::calibrateCb,
                                     this);
    }

  protected:
    bool calibrateCb(rbot_msgs::CalibrateSensorModel1D::Request &request_,
                     rbot_msgs::CalibrateSensorModel1D::Response &)
    {
        auto model = request_.model;
        if (model.validity.offset) {
            this->offset = model.offset;
        }
        if (model.validity.drift) {
            this->drift = model.drift;
        }
        if (model.validity.drift_frequency) {
            this->driftFreq = model.drift_frequency;
        }
        if (model.validity.scale_error) {
            this->scale = model.scale_error;
        }
        if (model.validity.gaussian_noise) {
            this->gaussianNoise = model.gaussian_noise;
        }
        return true;
    }

    ros::ServiceServer m_calibrateSrv;
};  // class RosSdfSensorModel<double>

template <>
class RosSdfSensorModel<gazebo::math::Vector3>
    : public SdfSensorModel<gazebo::math::Vector3>
{
  public:
    using SdfSensorModel<gazebo::math::Vector3>::SdfSensorModel;
    void setServices(std::unique_ptr<ros::NodeHandle> nhPtr_)
    {
        m_calibrateSrv = nhPtr_->advertiseService(
            "sensor_model/calibrate",
            &RosSdfSensorModel<gazebo::math::Vector3>::calibrateCb,
            this);
    }

  protected:
    bool calibrateCb(rbot_msgs::CalibrateSensorModel3D::Request &request_,
                     rbot_msgs::CalibrateSensorModel3D::Response &)
    {
        using rbot_utils::copy_3d;
        using rbot_utils::copy_3d;
        auto model = request_.model;
        if (model.validity.offset) {
            copy_3d(this->offset, model.offset);
        }
        if (model.validity.drift) {
            copy_3d(this->drift, model.drift);
        }
        if (model.validity.drift_frequency) {
            copy_3d(this->driftFreq, model.drift_frequency);
        }
        if (model.validity.scale_error) {
            copy_3d(this->scale, model.scale_error);
        }
        if (model.validity.gaussian_noise) {
            copy_3d(this->gaussianNoise, model.gaussian_noise);
        }
        return true;
    }

    ros::ServiceServer m_calibrateSrv;
};  // class RosSdfSensorModel<gazebo::math::Vector3>

template <>
void SensorModel<gazebo::math::Vector3>::reset()
{
    m_kernel.reset();
#if GAZEBO_MAJOR_VERSION >= 9
    currDrift.X() = m_kernel.generate<double>(0., currDrift.X());
    currDrift.Y() = m_kernel.generate<double>(0., currDrift.Y());
    currDrift.Z() = m_kernel.generate<double>(0., currDrift.Z());
#else
    currDrift.x = m_kernel.generate<double>(0., currDrift.x);
    currDrift.y = m_kernel.generate<double>(0., currDrift.y);
    currDrift.z = m_kernel.generate<double>(0., currDrift.z);
#endif
    m_currError = gazebo::math::Vector3();
}

template <>
gazebo::math::Vector3 SensorModel<gazebo::math::Vector3>::m_internalUpdate(
    double dt_)
{
    gazebo::math::Vector3 error;
#if GAZEBO_MAJOR_VERSION >= 9
    currDrift.X() = currDrift.X() * std::exp(-dt_ * driftFreq.X());
    currDrift.Y() = currDrift.Y() * std::exp(-dt_ * driftFreq.Y());
    currDrift.Z() = currDrift.Z() * std::exp(-dt_ * driftFreq.Z());

    currDrift.X() =
        currDrift.X() + dt_ * m_kernel.generate<double>(
                                  0., std::sqrt(2 * driftFreq.X()) * drift.X());
    currDrift.Y() =
        currDrift.Y() + dt_ * m_kernel.generate<double>(
                                  0., std::sqrt(2 * driftFreq.Y()) * drift.Y());
    currDrift.Z() =
        currDrift.Z() + dt_ * m_kernel.generate<double>(
                                  0., std::sqrt(2 * driftFreq.Z()) * drift.Z());

    error.X() = m_kernel.generate<double>(0., gaussianNoise.X());
    error.Y() = m_kernel.generate<double>(0., gaussianNoise.Y());
    error.Z() = m_kernel.generate<double>(0., gaussianNoise.Z());
#else
    currDrift.x = currDrift.x * std::exp(-dt_ * driftFreq.x);
    currDrift.y = currDrift.y * std::exp(-dt_ * driftFreq.y);
    currDrift.z = currDrift.z * std::exp(-dt_ * driftFreq.z);

    currDrift.x =
        currDrift.x + dt_ * m_kernel.generate<double>(
                                0., std::sqrt(2 * driftFreq.x) * drift.x);
    currDrift.y =
        currDrift.y + dt_ * m_kernel.generate<double>(
                                0., std::sqrt(2 * driftFreq.y) * drift.y);
    currDrift.z =
        currDrift.z + dt_ * m_kernel.generate<double>(
                                0., std::sqrt(2 * driftFreq.z) * drift.z);

    error.x = m_kernel.generate<double>(0., gaussianNoise.x);
    error.y = m_kernel.generate<double>(0., gaussianNoise.y);
    error.z = m_kernel.generate<double>(0., gaussianNoise.z);
#endif
    return offset + currDrift + error;
}
}  // namespace rbot_sim_integration
#endif  // _RBOT_SIM_INTEGRATION_GAZEBO_SDF_SENSOR_MODEL_HPP_
