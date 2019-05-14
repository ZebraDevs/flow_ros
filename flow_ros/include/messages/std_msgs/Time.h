/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 * 
 * @file Time.h
 */
#ifndef FLOW_ROS_MESSAGES_STD_MSGS_TIME_H
#define FLOW_ROS_MESSAGES_STD_MSGS_TIME_H

// ROS
#include <ros/time.h>

// Messages
#include <std_msgs/Time.h>

// Flow
#include <flow_ros/messages.h>

namespace flow_ros
{

// std_msgs/Time specialization of MessageDispatch
template<>
class MessageDispatch<const std_msgs::Time, true>
{
public:
  MessageDispatch() = default;
  MessageDispatch(const MessageDispatch&) = default;

  /**
   * @brief Message constructor (move enabled)
   */
  explicit MessageDispatch(typename std_msgs::Time::ConstPtr msg) :
    msg_{std::move(msg)}
  {}

  /**
   * @brief Returns sequencing stamp associated with data element
   */
  inline const ros::Time& stamp() const
  {
    return msg_->data;
  }

  /**
   * @brief Returns const reference to underlying data element
   */
  inline const typename std_msgs::Time::ConstPtr& data() const
  {
    return msg_;
  }

  /**
   * @brief LT relational overload
   */
  inline bool operator<(const MessageDispatch& other) const
  {
    return stamp() < other.stamp();
  }

private:
  /// Message payload
  typename std_msgs::Time::ConstPtr msg_;
};


// std_msgs/Time specialization of StampSetter
template<>
struct StampSetter<std_msgs::Time, true>
{
  /**
   * @brief Sets message stamp
   */
  inline void operator()(std_msgs::Time& msg, const ros::Time& stamp) const
  {
    msg.data = stamp;
  }

  /**
   * @brief Pass-through
   */
  inline void operator()(const std_msgs::Time& msg, const ros::Time& stamp) const {}
};

}  // namespace flow_ros

#endif  // FLOW_ROS_MESSAGES_STD_MSGS_TIME_H
