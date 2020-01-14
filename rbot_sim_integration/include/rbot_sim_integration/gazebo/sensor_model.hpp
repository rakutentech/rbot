#ifndef _RBOT_SIM_INTEGRATION_SENSOR_MODEL_HPP_
#define _RBOT_SIM_INTEGRATION_SENSOR_MODEL_HPP_

#include <cmath>
#include <memory>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265359
#endif

namespace rbot_sim_integration {
struct SensorModelGaussianKernel
{
    SensorModelGaussianKernel() : m_uniformKernel(0., 1.) {}

    void reset() { m_uniformKernel.reset(); }

    template <typename T>
    T generate(const T mu_, const T sigma_)
    {
        double u = m_uniformKernel(m_generator);
        double v = m_uniformKernel(m_generator);
        double x = std::sqrt(-2 * std::log(u)) * std::cos(2 * M_PI * v);
        return (sigma_ * x + mu_);
    }

  protected:
    std::default_random_engine m_generator;
    std::uniform_real_distribution<double> m_uniformKernel;
};

// add a int D for inter-axis interference?
/**
 * @class SensorModel
 * @template T typename of error
 * @brief stores modified data of a sensor based on a gaussian error model
 * @detail This is supposed to be a base class
 */
template <typename T>
class SensorModel
{
  public:
    /**
     * @ctor
     * @brief calls reset
     */
    SensorModel() : offset(T()), drift(T()), gaussianNoise(T()), currDrift(T())
    {
        // no other way to declare them coz of gazebo::math::Vector3
        driftFreq = 1.0 / 3600;
        scale = 1.0;
        reset(T());
    }

    /**
     * @dtor
     * @brief virtual for inheritance, otherwise empty
     */
    virtual ~SensorModel() {}

    /**
     * @fn operator()
     * @brief apply the sensor model without updating it
     * @param[in] value_ noise free measurement
     * @return noisy measurement
     */
    virtual T operator()(const T& value_) const
    {
        return value_ * scale + m_currError;
    }

    /**
     * @fn operator()
     * @brief apply the sensor model and update the error
     * @param[in] value_ noise free measurement
     * @param[in] dt_ timestep size
     * @return noisy measurement
     */
    virtual T operator()(const T& value_, double dt_)
    {
        return value_ * scale + update(dt_);
    }

    /**
     * @fn update
     * @brief update the error based on a timestep input
     * @param[in] dt_ timestep size
     * @return new error post update
     */
    virtual T update(double dt_);

    /**
     * @fn reset
     * @brief reset the sensor model
     * @detail
     *     * drift is derived from guassian distribution, mean 0, sigma of drift
     */
    virtual void reset();

    /**
     * @fn reset
     * @brief reset the sensor model
     * @param[in] value_ new value of error
     */
    virtual void reset(const T& value_);

    /**
     * @fn getCurrentError
     * @brief returns the current error
     */
    virtual const T& getCurrentError() const { return m_currError; }

    /**
     * @fn getCurrentBias
     * @brief returns the current sensor bias
     */
    virtual T getCurrentBias() const { return currDrift + offset; }

    ///< offset from zero, drift size, and noise strength
    T offset, drift, gaussianNoise;

    ///< current drift
    T currDrift;

    ///< driftFreq is measured in units per hour
    T driftFreq, scale;

  protected:
    virtual T m_internalUpdate(double dt_);

    T m_currError;
    SensorModelGaussianKernel m_kernel;
};  // class SensorModel

template <typename T>
void SensorModel<T>::reset()
{
    m_kernel.reset();
    m_currError = T();
    auto zero = m_currError[0];
    for (auto& drift : currDrift) {
        drift = m_kernel.generate<decltype(zero)>(zero, drift);
    }
}

template <>
void SensorModel<double>::reset()
{
    m_kernel.reset();
    currDrift = m_kernel.generate<double>(0., currDrift);
    m_currError = 0;
}

template <typename T>
void SensorModel<T>::reset(const T& value_)
{
    m_kernel.reset();
    currDrift = value_;
    m_currError = T();
}

template <typename T>
T SensorModel<T>::update(double dt_)
{
    m_currError = m_internalUpdate(dt_);
    return m_currError;
}

template <typename T>
T SensorModel<T>::m_internalUpdate(double dt_)
{
    T error;
    auto zero = error[0];
    for (std::size_t i = 0; i < currDrift.size(); ++i) {
        currDrift[i] = currDrift[i] * std::exp(-dt_ * driftFreq[i]);
        currDrift[i] = currDrift[i] +
                       dt_ * m_kernel.generate<decltype(zero)>(
                                 zero, std::sqrt(2 * driftFreq[i]) * drift[i]);
        error[i] = m_kernel.generate<decltype(zero)>(zero, gaussianNoise[i]);
    }
    return offset + currDrift + error;
}

template <>
double SensorModel<double>::m_internalUpdate(double dt_)
{
    currDrift = currDrift * std::exp(-dt_ * driftFreq);
    currDrift = currDrift + dt_ * m_kernel.generate<double>(
                                      0., std::sqrt(2 * driftFreq) * drift);
    return offset + currDrift + m_kernel.generate<double>(0., gaussianNoise);
}
}  // namespace rbot_sim_integration
#endif  // _RBOT_SIM_INTEGRATION_SENSOR_MODEL_HPP_
