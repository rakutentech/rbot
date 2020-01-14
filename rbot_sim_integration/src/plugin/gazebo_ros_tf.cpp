#include <rbot_sim_integration/plugin/gazebo_ros_tf.hpp>

#include <rbot_utils/algorithm.hpp>

namespace gazebo {
GazeboRosTf::GazeboRosTf() : GazeboModelPluginBase("rbot_tf_publisher") {}

void GazeboRosTf::m_load()
{
    setFromSdf(m_sdfPtr, "publish", m_publish, m_publish);
    setFromSdf(m_sdfPtr, "childFrame", m_childFrame, m_childFrame);
    m_odom.header.frame_id = "map";
    m_odom.child_frame_id = m_modelName + "/" + m_childFrame;
}

void GazeboRosTf::m_onUpdate() { m_prepareTf(); }

void GazeboRosTf::m_spinOnce() { m_tfBroadcaster.sendTransform(m_odom); }

void GazeboRosTf::m_prepareTf()
{
    if (m_publish) {
        // TODO: is this time wrong??
        m_odom.header.stamp = ros::Time::now();

#if GAZEBO_MAJOR_VERSION >= 9
        auto pose = m_parentPtr->WorldPose();
        rbot_utils::copy_3d(m_odom.transform.translation, pose.Pos());
        rbot_utils::copy_4d(m_odom.transform.rotation, pose.Rot());
#else
        auto pose = m_parentPtr->GetWorldPose();
        rbot_utils::copy_3d(m_odom.transform.translation, pose.pos);
        rbot_utils::copy_4d(m_odom.transform.rotation, pose.rot);
#endif
    }
}
}  // namespace gazebo
