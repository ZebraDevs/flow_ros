/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file  ros_subscription_wrapper.h
 */
#ifndef FLOW_ROS_ROUTING_ROS_PUBLICATION_H
#define FLOW_ROS_ROUTING_ROS_PUBLICATION_H

// C++ Standard Library
#include <string>

// ROS
#include <ros/publisher.h>

// Flow
#include <flow_ros/routing/publication_wrapper.h>
#include <flow_ros/routing/transport_info.h>

namespace flow_ros
{
namespace routing
{

/**
 * @brief Wrapper object for a <code>ros::Publisher</code>
 *
 * @tparam MsgT  message data resource type
 */
template<typename MsgT>
class ROSPublication : public PublicationWrapper<MsgT>
{
public:
  /**
   * @brief ROS publisher constructor
   *
   * @param pub  ROS publisher encapsulation object
   */
  explicit ROSPublication(const ros::Publisher& pub) :
    pub_{pub}
  {}

  /**
   * @brief Call all held subscriber callbacks on message being published
   *
   * @param message  message data to publish
   */
  void publish(const message_shared_ptr_t<MsgT>& message) const final
  {
    pub_.publish(message);
  }

  /**
   * @brief Returns topic associated with publication
   */
  std::string getTopic() const final
  {
    return pub_.getTopic();
  }

  /**
   * @brief Returns number of local subscriptions connected to this LocalPublication
   */
  std::uint32_t getNumSubscribers() const final
  {
    return pub_.getNumSubscribers();
  }

  /**
   * @brief Returns transport method (code) associated with this publisher
   */
  TransportMethod getTransportMethod() const final
  {
    return TransportMethod::ROS;
  }

  /**
   * @brief Validation cast operator
   * @retval true  if underlying ROS publisher is valid
   * @retval false  otherwise
   */
  bool isValid() const final
  {
    return static_cast<bool>(pub_);
  }

private:
  /// Underlying ROS publisher
  ros::Publisher pub_;
};

}  // namespace routing
}  // namespace flow_ros

#endif  // FLOW_ROS_ROUTING_ROS_PUBLICATION_H
