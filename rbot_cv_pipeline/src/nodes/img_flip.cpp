#include <rbot_cv_pipeline/img_flip.hpp>
#include <rbot_utils/node.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "img_flip_node");

    rbot_cv_pipeline::ImgFlip<rbot_utils::Node<>> flipper;
    flipper.init();

    ros::spin();
    return 0;
}
