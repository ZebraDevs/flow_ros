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
template <typename MsgT> class ROSPublication final : public PublicationWrapper<MsgT>
{
public:
  /**
   * @brief ROS publisher constructor
   *
   * @param pub  ROS publisher encapsulation object
   */
  explicit ROSPublication(const ros::Publisher& pub) : pub_{pub} {}

  /**
   * @brief Call all held subscriber callbacks on message being published
   *
   * @param message  message data to publish
   */
  void publish(const message_shared_const_ptr_t<MsgT>& message) const override { pub_.publish(message); }

  /**
   * @copydoc PublicationWrapper::getTopic
   */
  std::string getTopic() const override { return pub_.getTopic(); }

  /**
   * @copydoc PublicationWrapper::getNumSubscribers
   */
  std::uint32_t getNumSubscribers() const override { return pub_.getNumSubscribers(); }

  /**
   * @copydoc PublicationWrapper::isLatched
   */
  bool isLatched() const override { return pub_.isLatched(); }

  /**
   * @copydoc PublicationWrapper::getTransportMethod
   */
  TransportMethod getTransportMethod() const override { return TransportMethod::ROS; }

  /**
   * @copydoc PublicationWrapper::isValid
   *
   * @retval true  if underlying ROS publisher is valid
   * @retval false  otherwise
   */
  bool isValid() const override { return static_cast<bool>(pub_); }

private:
  /// Underlying ROS publisher
  ros::Publisher pub_;
};

}  // namespace routing
}  // namespace flow_ros

#endif  // FLOW_ROS_ROUTING_ROS_PUBLICATION_H
