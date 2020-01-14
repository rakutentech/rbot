#ifndef _RBOT_CV_PIPELINE_CVT_COLOR_CPP_
#define _RBOT_CV_PIPELINE_CVT_COLOR_CPP_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <rbot_cv_pipeline/cvt_color.hpp>

namespace rbot_cv_pipeline {
using CvtColorNodelet = CvtColor<nodelet::Nodelet>;
}  // namespace rbot_cv_pipeline

PLUGINLIB_EXPORT_CLASS(rbot_cv_pipeline::CvtColorNodelet, nodelet::Nodelet);
#endif /* _RBOT_CV_PIPELINE_CVT_COLOR_CPP_ */
