#ifndef _RBOT_UTILS_V1_ROS_UTILS_HPP_
#define _RBOT_UTILS_V1_ROS_UTILS_HPP_

// std
#include <algorithm>
#include <functional>
#include <optional>
#include <vector>

#ifndef RBOT_SKIP_IMAGES
// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>

// opencv
#include <opencv2/core.hpp>
#endif

namespace rbot_utils {
namespace v1 {
/**
 * @brief publish after checking for subscribers, works with at least
 * {ros, image_transport}::Publisher
 * @tparam force_publish if true, skips check for subscribers
 * @details Useful for publishers with expensive operations like pointclouds and
 * images. For never skipping the check, @see publish<Publisher, Data>
 */
template <bool force_publish, class Publisher, class Data>
void publish(const Publisher& pub_, const Data& data_)
{
    if (force_publish || pub_.getNumSubscribers()) {
        pub_.publish(data_);
    }
}

/**
 * @brief checks for subscribers and then publishes
 * @details @see publish<bool, Publisher, Data>
 */
template <class Publisher, class Data>
void publish(const Publisher& pub_, const Data& data_)
{
    publish<false>(pub_, data_);
}

/**
 * @brief get subscribers of a publisher
 */
template <class Publisher>
std::uint32_t getNumSubscribers(const Publisher& pub_)
{
    return pub_.getNumSubscribers();
}

/**
 * @brief get subscribers of a publisher
 */
template <class Subscriber>
void shutdown(Subscriber& sub_)
{
    sub_.shutdown();
}

#ifndef RBOT_SKIP_IMAGES
/**
 * @brief convert cv::Mat to sensor_msgs::Image
 */
static sensor_msgs::Image::Ptr matToMsg(const std_msgs::Header& header_,
                                        const std::string& encoding_,
                                        const cv::Mat& data_)
{
    return cv_bridge::CvImage(header_, encoding_, data_).toImageMsg();
}

/**
 * @brief convert sensor_msgs::Image to std::optional<cv::Mat>
 */
static std::optional<cv::Mat> msgToMat(const sensor_msgs::Image::ConstPtr& msg_)
{
    try {
        return cv_bridge::toCvShare(msg_, msg_->encoding)->image;
    } catch (cv::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to '%s'.",
                  msg_->encoding.c_str(),
                  msg_->encoding.c_str());
        return {};
    }
}
#endif
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_ROS_UTILS_HPP_ */
