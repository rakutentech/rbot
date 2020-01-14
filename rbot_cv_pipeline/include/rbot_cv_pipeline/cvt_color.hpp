#ifndef _RBOT_CV_PIPELINE_CVT_COLOR_HPP_
#define _RBOT_CV_PIPELINE_CVT_COLOR_HPP_

// std
#include <mutex>
#include <optional>
#include <string>

// opencv
#include <opencv2/imgproc/imgproc.hpp>

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// library
#include <rbot_utils/reactive_node.hpp>
#include <rbot_utils/ros_utils.hpp>

namespace rbot_cv_pipeline {
template <class BaseClass>
struct CvtColor : public rbot_utils::ReactiveNodeInterface<BaseClass>
{
  protected:
    using Self = CvtColor<BaseClass>;
    using Base = rbot_utils::ReactiveNodeInterface<BaseClass>;

  public:
    void onInit() final
    {
        namespace ph = std::placeholders;
        ros::NodeHandle& nh = Base::getNodeHandle();
        ros::NodeHandle& priv_nh = Base::getPrivateNodeHandle();

        nh.param<int>(priv_nh.resolveName("cv_code"), m_cvCode, CV_GRAY2BGR);
        nh.param<std::string>(
            priv_nh.resolveName("output_encoding"), m_outEncoding, "bgr8");

        const auto img_event_cb = Base::getImgEventCb();
        image_transport::ImageTransport& it = Base::getImageTransport();

        Base::imgPub.push_back(
            it.advertise(nh.resolveName("image_raw"), 1, img_event_cb, img_event_cb));
    }

    void setupSubscribers() final
    {
        // @TODO: needs a better interface than this
        Base::imgSub.resize(1);
        ros::NodeHandle& nh = Base::getNodeHandle();
        const std::string name = nh.resolveName("input");
        Base::subscribe(Base::imgSub[0], name, m_qSize, &Self::m_imageCb, this);
    }

  protected:
    int m_cvCode = 0;
    std::string m_outEncoding = "";
    std::int32_t m_qSize = 5;

    void m_imageCb(const sensor_msgs::ImageConstPtr& msg_)
    {
        const auto in_image = rbot_utils::msgToMat(msg_);
        if (!in_image) {
            return;
        }

        cv::Mat out_image;
        cv::cvtColor(in_image.value(), out_image, m_cvCode);
        rbot_utils::publish(
            Base::imgPub[0],
            rbot_utils::matToMsg(msg_->header, m_outEncoding, out_image));
    }
};
}  // namespace rbot_cv_pipeline
#endif /* _RBOT_CV_PIPELINE_CVT_COLOR_HPP_ */
