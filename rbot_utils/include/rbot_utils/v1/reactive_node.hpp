#ifndef _RBOT_UTILS_V1_REACTIVE_NODE_HPP_
#define _RBOT_UTILS_V1_REACTIVE_NODE_HPP_

// std
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

// ros
#include <image_transport/image_transport.h>
#include <ros/ros.h>

// library
#include <rbot_utils/ros_utils.hpp>

namespace rbot_utils {
namespace v1 {
template <class BaseClass>
struct ReactiveNodeInterface : public BaseClass
{
  protected:
    using Base = BaseClass;
    using Self = ReactiveNodeInterface<Base>;

  public:
    ReactiveNodeInterface() = default;
    virtual ~ReactiveNodeInterface() = default;

    image_transport::ImageTransport& getImageTransport()
    {
        // ImageTransport can't be created until NodeHandle is active
        // which requires init to be called
        if (!m_init) {
            m_initIt();
        }
        return *m_itPtr;
    }

    image_transport::ImageTransport& getPrivateImageTransport()
    {
        if (!m_init) {
            m_initIt();
        }
        return *m_privItPtr;
    }

    /**
     * @brief callback to shutdown/awaken subscribers based on publisher status.
     * Prefer to use getRosEventCb or getImgEventCb instead
     * @details Assumes that all subscribers are required for even 1 topic
     * being subscribed. All subscribers are shutdown when the node becomes a
     * pure data sink, ie:
     * * ros and image_transport subscribers count is 0 (zero)
     * * not a single publisher has any subscriber
     * In this case, if publishers should still publish, please provide your own
     * event callbacks
     */
    template <class SingleSubscriberPublisher>
    auto getEventCb()
    {
        namespace ph = std::placeholders;
        return std::bind(
            &Self::m_connectCb<SingleSubscriberPublisher>, this, ph::_1);
    }

    ros::SubscriberStatusCallback getRosEventCb()
    {
        return getEventCb<ros::SingleSubscriberPublisher>();
    }

    image_transport::SubscriberStatusCallback getImgEventCb()
    {
        return getEventCb<image_transport::SingleSubscriberPublisher>();
    }

    /**
     * @bief setup subscribers each time publishers are connected to something
     * @details Use subscribe<T, M> to prevent reinit of active subscribers
     */
    virtual void setupSubscribers() = 0;

    // doesn't make sense to provide advertise because
    // * subscriber connect/disconnect callbacks are specialized
    // * event_cb are already provided for the user

    /**
     * @brief almost a drop in replacement for ros::NodeHandle::subscribe
     * @details special subscriber calls to
     * * create subscriber if one doesn't exist
     * * do nothing if subscriber is already active
     * @attention only one version for each {ros, image_transport} is provided
     * If needed, please create PR for more
     */
    template <class T, class M>
    void subscribe(ros::Subscriber& sub_,
                   std::string topicName_,
                   std::int32_t queueSize_,
                   void (T::*fp_)(M),
                   T* obj_,
                   ros::TransportHints transportHints_ = {})
    {
        ros::NodeHandle& nh = Base::getNodeHandle();
        if (static_cast<void*>(sub_)) {
            return;
        }
        sub_ = nh.subscribe(topicName_, queueSize_, fp_, obj_, transportHints_);
    }

    template <class T>
    void subscribe(image_transport::Subscriber& sub_,
                   std::string topicName_,
                   std::int32_t queueSize_,
                   void (T::*fp_)(const sensor_msgs::ImageConstPtr&),
                   T* obj_,
                   image_transport::TransportHints transportHints_ = {})
    {
        image_transport::ImageTransport& it = getImageTransport();
        if (static_cast<void*>(sub_)) {
            return;
        }
        sub_ = it.subscribe(topicName_, queueSize_, fp_, obj_, transportHints_);
    };

    template <class T>
    void subscribeCamera(image_transport::Subscriber& sub_,
                         std::string baseTopicName_,
                         std::int32_t queueSize_,
                         void (T::*fp_)(const sensor_msgs::ImageConstPtr&,
                                        const sensor_msgs::CameraInfoConstPtr&),
                         T* obj_,
                         image_transport::TransportHints transportHints_ = {})
    {
        image_transport::ImageTransport& it = getImageTransport();
        if (static_cast<void*>(sub_)) {
            return;
        }
        sub_ = it.subscribe(
            baseTopicName_, queueSize_, fp_, obj_, transportHints_);
    };

    // list of all subsribers and publishers
    std::vector<ros::Subscriber> rosSub;
    std::vector<ros::Publisher> rosPub;
    std::vector<image_transport::Subscriber> imgSub;
    std::vector<image_transport::Publisher> imgPub;

  private:
    using RosSub = ros::Subscriber;
    using RosPub = ros::Publisher;
    using ImgSub = image_transport::Subscriber;
    using ImgPub = image_transport::Publisher;

    std::unique_ptr<image_transport::ImageTransport> m_itPtr, m_privItPtr;
    std::mutex m_connectMutex;
    bool m_init = false;

    void m_initIt()
    {
        m_itPtr = std::make_unique<image_transport::ImageTransport>(
            Base::getNodeHandle());
        m_privItPtr = std::make_unique<image_transport::ImageTransport>(
            Base::getPrivateNodeHandle());
        m_init = true;
    }

  protected:
    template <class SingleSubscriberPublisher>
    void m_connectCb(const SingleSubscriberPublisher&)
    {
        const auto rosSubscribed = std::any_of(
            rosPub.begin(), rosPub.end(), getNumSubscribers<RosPub>);
        const auto imgSubscribed = std::any_of(
            imgPub.begin(), imgPub.end(), getNumSubscribers<ImgPub>);
        ROS_INFO("%d, %d", rosSubscribed, imgSubscribed);
        const auto subscribed = rosSubscribed || imgSubscribed;
        if (subscribed == false) {
            std::for_each(rosSub.begin(), rosSub.end(), shutdown<RosSub>);
            std::for_each(imgSub.begin(), imgSub.end(), shutdown<ImgSub>);
            return;
        };
        setupSubscribers();
    }
};
}  // namespace v1
}  // namespace rbot_utils

#endif /* ifndef _RBOT_UTILS_V1_REACTIVE_NODE_HPP_ */

