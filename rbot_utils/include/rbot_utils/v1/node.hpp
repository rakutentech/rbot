#ifndef _RBOT_UTILS_V1_NODE_HPP_
#define _RBOT_UTILS_V1_NODE_HPP_

#include <ros/ros.h>

#include <rbot_utils/macro.hpp>

namespace rbot_utils {
namespace v1 {
namespace detail {
SETUP_HAS_PUBLIC_TYPE_ALIAS(nodehandle_t);
}  // namespace detail

template <class Node, class = void>
struct node_traits
{
    using nodehandle_t = ros::NodeHandle;
};

template <class Node>
struct node_traits<Node, detail::HasPublicTypeAlias_nodehandle_t<Node>>
{
    using nodehandle_t = typename Node::nodehandle_t;
};

template <class NodeHandle = ros::NodeHandle>
struct Node
{
    using nodehandle_t = NodeHandle;

    NodeHandle& getNodeHandle() { return m_nh; }
    NodeHandle& getPrivateNodeHandle() { return m_privNh; }

    const std::string& getName() const { return ros::this_node::getName(); }

    std::string getSuffixedName(const std::string& suffix_) const
    {
        return getName() + "." + suffix_;
    }

    Node() = default;
    virtual ~Node() = default;

    virtual void onInit() = 0;

    /**
     * @brief provided to make "node" independent from ros::init
     * @detail brings the ROS1 node closer to the independent nodes in ROS2
     */
    void init() { onInit(); }

    /**
     * @brief ALMOST a drop-in replacement for ros.nodelet::Nodelet
     * @detail argv_, qSingleThread_ and qMultiThread_ are ignored
     */
    void init(const std::string& name_,
              const ros::M_string& remappingArgs_,
              const ros::V_string& /*argv_*/ = {},
              ros::CallbackQueueInterface* /*qSingleThread_*/ = nullptr,
              ros::CallbackQueueInterface* /*qMultiThread_*/ = nullptr)
    {
        m_privNh = ros::NodeHandle(name_, remappingArgs_);
        m_nh =
            ros::NodeHandle(ros::names::parentNamespace(name_), remappingArgs_);
    }

  private:
    NodeHandle m_nh, m_privNh = {"~"};
};

static_assert(detail::has_public_type_alias_nodehandle_t_v<Node<>>);
}  // namespace v1
}  // namespace rbot_utils

#endif /* ifndef _RBOT_UTILS_V1_NODE_HPP_ */
