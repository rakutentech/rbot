#include <rbot_utils/node.hpp>

#include <rbot_cv_pipeline/image_splitter.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_splitter_node");

    rbot_cv_pipeline::ImageSplitter<2, rbot_utils::Node<>> splitter;
    splitter.init();

    ros::spin();
    return 0;
}

