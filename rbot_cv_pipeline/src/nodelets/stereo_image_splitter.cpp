#ifndef _RBOT_CV_PIPELINE_NODELET_IMAGE_SPLITTER_CPP_
#define _RBOT_CV_PIPELINE_NODELET_IMAGE_SPLITTER_CPP_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <rbot_cv_pipeline/image_splitter.hpp>

namespace rbot_cv_pipeline {
using StereoImageSplitterNodelet = ImageSplitter<2, nodelet::Nodelet>;
}  // namespace rbot_cv_pipeline

PLUGINLIB_EXPORT_CLASS(rbot_cv_pipeline::StereoImageSplitterNodelet,
                       nodelet::Nodelet);

#endif  /* _RBOT_CV_PIPELINE_NODELET_IMAGE_SPLITTER_CPP_ */
