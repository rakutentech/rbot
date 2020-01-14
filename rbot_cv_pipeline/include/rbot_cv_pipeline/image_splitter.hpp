#ifndef _RBOT_CV_PIPELINE_IMAGE_SPLITTER_HPP_
#define _RBOT_CV_PIPELINE_IMAGE_SPLITTER_HPP_

// std
#include <algorithm>
#include <functional>
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
template <std::size_t N, class BaseClass>
struct ImageSplitter : public rbot_utils::ReactiveNodeInterface<BaseClass>
{
  protected:
    using Self = ImageSplitter<N, BaseClass>;
    using Base = rbot_utils::ReactiveNodeInterface<BaseClass>;

  public:
    void onInit() final
    {
        ros::NodeHandle& nh = Base::getNodeHandle();
        image_transport::ImageTransport& it = Base::getImageTransport();

        auto img_event_cb = Base::getImgEventCb();

        for (std::size_t i = 0; i < N; ++i) {
            const std::string baseName =
                nh.resolveName("camera_" + std::to_string(i));
            Base::imgPub.push_back(it.advertise(
                baseName + "/image_raw", 1, img_event_cb, img_event_cb));
        }
    }

    void setupSubscribers() final
    {
        Base::imgSub.resize(1);
        ros::NodeHandle& nh = Base::getNodeHandle();
        const std::string name = nh.resolveName("image");
        Base::subscribe(Base::imgSub[0], name, m_qSize, &Self::m_imageCb, this);
    }

  protected:
    // camera_info not required since it's same for both cameras
    std::int32_t m_qSize = 5;

    void m_imageCb(const sensor_msgs::ImageConstPtr& msg_)
    {
        const auto inImage = rbot_utils::msgToMat(msg_);
        if (!inImage) {
            return;
        }

        std::array<cv::Mat, N> outImage;

        auto width = inImage.value().size().width / N;
        // assert((size.width * m_outputPub.size()) == in_image.size().width)

        for (std::size_t i = 0, offset = 0; i < N; i++, offset += width) {
            outImage[i] = inImage.value()(cv::Range::all(),
                                          cv::Range(offset, offset + width));
        }

        for (std::size_t i = 0; i < N; ++i) {
            const auto& msg =
                rbot_utils::matToMsg(msg_->header, msg_->encoding, outImage[i]);
            rbot_utils::publish(Base::imgPub[i], msg);
        }
    }
};
}  // namespace rbot_cv_pipeline
#endif /* ifndef _RBOT_CV_PIPELINE_IMAGE_SPLITTER_HPP_ */
