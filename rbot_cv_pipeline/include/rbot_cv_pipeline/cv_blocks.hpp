#ifndef _RBOT_CV_PIPELINE_CV_BLOCKS_HPP_
#define _RBOT_CV_PIPELINE_CV_BLOCKS_HPP_

// std
#include <vector>

// opencv
#include <opencv2/imgproc/imgproc.hpp>

namespace rbot_cv_pipeline {
std::vector<cv::Mat> extract_slices(const std::vector<cv::Mat>& images,
                                    const cv::Rect& roi)
{
    std::vector<cv::Mat> slices(images.size());
    std::transform(
        images.cbegin(), images.cend(), slices.begin(), [&](const auto& img) {
            return img(roi);
        });
    return slices;
}

std::vector<cv::Mat> extract_slices(const cv::Mat& image, const cv::Rect& roi)
{
    std::vector<cv::Mat> slice(1, image(roi));
    return slice;
}

std::vector<cv::Mat> extract_slices(const std::vector<cv::Mat>& images,
                                    const int xmin,
                                    const int xmax,
                                    const int ymin,
                                    const int ymax)
{
    return extract_slices(images,
                          cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin));
}

std::vector<cv::Mat> extract_slices(const cv::Mat& image,
                                    const int xmin,
                                    const int xmax,
                                    const int ymin,
                                    const int ymax)
{
    return extract_slices(image,
                          cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin));
}

cv::Mat cvt_hsv(const cv::Mat& image)
{
    cv::Mat out_img;
    cv::cvtColor(image, out_img, CV_RGB2HSV);
    return out_img;
}

cv::Mat cvt_lab(const cv::Mat& image)
{
    cv::Mat out_img;
    cv::cvtColor(image, out_img, CV_RGB2Lab);
    return out_img;
}

cv::Mat color_threshold(const cv::Mat& hsv_image,
                        const cv::Scalar& hsv_min,
                        const cv::Scalar& hsv_max,
                        const cv::Mat& lab_image,
                        const cv::Scalar& lab_min,
                        const cv::Scalar& lab_max)
{
    cv::Mat lab_mask;
    cv::inRange(lab_image, lab_min, lab_max, lab_mask);

    cv::Mat hsv_mask;
    if (hsv_min[0] > hsv_max[0]) {
        cv::Mat hsv_lower_mask, hsv_upper_mask;
        const cv::Scalar hsv_lower_lower(0, hsv_min[1], hsv_min[2]);
        const cv::Scalar hsv_upper_upper(180, hsv_max[1], hsv_max[2]);
        cv::inRange(hsv_image, hsv_lower_lower, hsv_max, hsv_lower_mask);
        cv::inRange(hsv_image, hsv_max, hsv_upper_upper, hsv_upper_mask);
        hsv_mask = hsv_lower_mask | hsv_upper_mask;
    } else {
        cv::inRange(hsv_image, hsv_min, hsv_max, hsv_mask);
    }
    return hsv_mask & lab_mask;
}
}  // namespace rbot_cv_pipeline
#endif /* _RBOT_CV_PIPELINE_CV_BLOCKS_HPP_ */
