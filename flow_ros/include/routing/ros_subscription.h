/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file  ros_subscription_wrapper.h
 */
#ifndef FLOW_ROS_ROUTING_ROS_SUBSCRIPTION_H
#define FLOW_ROS_ROUTING_ROS_SUBSCRIPTION_H

// C++ Standard Library
#include <string>

// ROS
#include <ros/subscriber.h>

// Flow
#include <flow_ros/routing/subscription_wrapper.h>


namespace flow_ros
{
namespace routing
{

/**
 * @brief Wrapper object for a <code>ros::Subscriber</code>
 */
class ROSSubscription final : public SubscriptionWrapper
{
public:
  /**
   * @brief ROS subscription constructor
   *
   * @param sub  ROS subscription encapsulation object
   */
  explicit ROSSubscription(const ros::Subscriber& sub) : sub_{sub} {}

  /**
   * @copydoc SubscriptionWrapper::getTopic
   */
  std::string getTopic() const override { return sub_.getTopic(); }

  /**
   * @copydoc SubscriptionWrapper::getNumPublishers
   */
  std::uint32_t getNumPublishers() const override { return sub_.getNumPublishers(); }

  /**
   * @copydoc SubscriptionWrapper::getTransportMethod
   */
  TransportMethod getTransportMethod() const override { return TransportMethod::ROS; }

  /**
   * @copydoc SubscriptionWrapper::isValid
   *
   * @retval true  if underlying ROS subscriber is valid
   * @retval false  otherwise
   */
  bool isValid() const override { return static_cast<bool>(sub_); }

private:
  /// Underlying ROS subscription
  ros::Subscriber sub_;
};

}  // namespace routing
}  // namespace flow_ros


#endif  // FLOW_ROS_ROUTING_ROS_SUBSCRIPTION_H
