#include <rbot_cv_pipeline/cvt_color.hpp>
#include <rbot_utils/node.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cvt_color_node");

    rbot_cv_pipeline::CvtColor<rbot_utils::Node<>> converter;
    converter.init();

    ros::spin();
    return 0;
}
