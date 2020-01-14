#include <rbot_sim_integration/gazebo/gazebo_ros_utils.hpp>

namespace gazebo {
std::string GetRobotNamespace(const physics::ModelPtr &model_,
                              const sdf::ElementPtr &sdf_,
                              const char *pInfo_)
{
    std::string ns;
    std::stringstream ss;
    if (sdf_->HasElement("robotNamespace")) {
        ns = sdf_->Get<std::string>("robotNamespace");
        if (ns.empty()) {
            ns = model_->GetScopedName();
            ss << "The 'robotNamespace' param was empty. ";
        }
        ss << "Value: " << ns;
    }
    if (pInfo_ != nullptr) {
        ROS_INFO_NAMED("utils", "%s Plugin: %s", pInfo_, ss.str().c_str());
    }
    return ns;
}
}  // namespace gazebo
