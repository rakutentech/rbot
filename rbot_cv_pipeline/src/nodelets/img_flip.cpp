#ifndef _RBOT_CV_PIPELINE_IMG_FLIP_CPP_
#define _RBOT_CV_PIPELINE_IMG_FLIP_CPP_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <rbot_cv_pipeline/img_flip.hpp>

namespace rbot_cv_pipeline {
using ImgFlipNodelet = ImgFlip<nodelet::Nodelet>;
}  // namespace rbot_cv_pipeline

PLUGINLIB_EXPORT_CLASS(rbot_cv_pipeline::ImgFlipNodelet, nodelet::Nodelet);
#endif /* _RBOT_CV_PIPELINE_IMG_FLIP_CPP_ */
