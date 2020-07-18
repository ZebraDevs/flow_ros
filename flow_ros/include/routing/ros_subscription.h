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
class ROSSubscription :  public SubscriptionWrapper
{
public:
  /**
   * @brief ROS subscription constructor
   *
   * @param sub  ROS subscription encapsulation object
   */
  explicit ROSSubscription(const ros::Subscriber& sub) :
    sub_{sub}
  {}

  /**
   * @brief Returns topic name associated with this object
   */
  inline std::string getTopic() const
  {
    return sub_.getTopic();
  }

  /**
   * @brief Returns number of local publications connected to this subscriber
   */
  inline std::uint32_t getNumPublishers() const
  {
    return sub_.getNumPublishers();
  }

  /**
   * @brief Returns transport method (code) associated with this subscriber
   */
  TransportMethod getTransportMethod() const final
  {
    return TransportMethod::ROS;
  }

  /**
   * @brief Validation cast operator
   * @retval true  if underlying ROS subscriber is valid
   * @retval false  otherwise
   */
  bool isValid() const final
  {
    return static_cast<bool>(sub_);
  }

private:
  /// Underlying ROS subscription
  ros::Subscriber sub_;
};

}  // namespace routing
}  // namespace flow_ros


#endif  // FLOW_ROS_ROUTING_ROS_SUBSCRIPTION_H
