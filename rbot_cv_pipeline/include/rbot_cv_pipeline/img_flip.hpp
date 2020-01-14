#ifndef _RBOT_CV_PIPELINE_IMG_FLIP_HPP_
#define _RBOT_CV_PIPELINE_IMG_FLIP_HPP_

// std
#include <mutex>
#include <optional>
#include <string>

// opencv
#include <opencv2/imgproc/imgproc.hpp>

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

// library
#include <rbot_utils/reactive_node.hpp>
#include <rbot_utils/ros_utils.hpp>

namespace rbot_cv_pipeline {
enum class FlipRotate : std::int8_t
{
    Vertically = 0,
    Horizontally = 1,
    Both = -1,
    CW180 = Both,
    CCW180 = Both,
    CW90 = 2,
    CCW90 = 3,
    None = -2
};

template <class BaseClass>
struct ImgFlip : public rbot_utils::ReactiveNodeInterface<BaseClass>
{
  protected:
    using Self = ImgFlip<BaseClass>;
    using Base = rbot_utils::ReactiveNodeInterface<BaseClass>;

  public:
    void onInit() final
    {
        ros::NodeHandle& nh = Base::getNodeHandle();
        ros::NodeHandle& priv_nh = Base::getPrivateNodeHandle();
        image_transport::ImageTransport it = Base::getImageTransport();

        // ros overloads are for int, not int32_t :/
        int flipMode;
        nh.param<int>(priv_nh.resolveName("flip_mode"),
                      flipMode,
                      static_cast<int>(FlipRotate::None));
        m_flipMode = static_cast<FlipRotate>(flipMode);

        const auto img_event_cb = Base::getImgEventCb();

        Base::imgPub.push_back(it.advertise(
            nh.resolveName("image_raw"), 1, img_event_cb, img_event_cb));
    }

    void setupSubscribers() final
    {
        Base::imgSub.resize(1);
        ros::NodeHandle& nh = Base::getNodeHandle();
        const std::string name = nh.resolveName("image");
        Base::subscribe(Base::imgSub[0], name, m_qSize, &Self::m_imageCb, this);
    }

  protected:
    FlipRotate m_flipMode = FlipRotate::None;
    std::int32_t m_qSize = 5;

    void m_imageCb(const sensor_msgs::ImageConstPtr& msg_)
    {
        auto inImage = rbot_utils::msgToMat(msg_);
        if (!inImage) {
            return;
        }

        auto& image = inImage.value();

        switch (m_flipMode) {
            case FlipRotate::CCW90:
                cv::transpose(image, image);
                [[fallthrough]];
            case FlipRotate::Vertically:
                cv::flip(image, image, 0);
                break;

            case FlipRotate::CW90:
                cv::transpose(image, image);
                [[fallthrough]];
            case FlipRotate::Horizontally:
                cv::flip(image, image, 1);
                break;

            case FlipRotate::Both:
                cv::flip(image, image, -1);
                break;

            default:
                ROS_WARN_STREAM_THROTTLE_NAMED(
                    5,
                    BaseClass::getName(),
                    "Unknown value for operation: "
                        << static_cast<int>(m_flipMode));
                [[fallthrough]];
            case FlipRotate::None:
                break;
        }

        rbot_utils::publish(
            Base::imgPub[0],
            rbot_utils::matToMsg(msg_->header, msg_->encoding, image));
    }
};
}  // namespace rbot_cv_pipeline
#endif /* _RBOT_CV_PIPELINE_IMG_FLIP_HPP_ */
