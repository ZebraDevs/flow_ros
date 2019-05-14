/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 * 
 * @file Header.h
 */
#ifndef FLOW_ROS_MESSAGES_STD_MSGS_HEADER_H
#define FLOW_ROS_MESSAGES_STD_MSGS_HEADER_H

// ROS
#include <ros/time.h>

// Messages
#include <std_msgs/Header.h>

// Flow
#include <flow_ros/messages.h>

namespace flow_ros
{

// std_msgs/Header specialization of MessageDispatch
template<>
class MessageDispatch<const std_msgs::Header, true>
{
public:
  MessageDispatch() = default;
  MessageDispatch(const MessageDispatch&) = default;

  /**
   * @brief Message constructor (move enabled)
   */
  explicit MessageDispatch(typename std_msgs::Header::ConstPtr msg) :
    msg_{std::move(msg)}
  {}

  /**
   * @brief Returns sequencing stamp associated with data element
   */
  inline const ros::Time& stamp() const
  {
    return msg_->stamp;
  }

  /**
   * @brief Returns const reference to underlying data element
   */
  inline const typename std_msgs::Header::ConstPtr& data() const
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
  typename std_msgs::Header::ConstPtr msg_;
};


// std_msgs/Header specialization of StampSetter
template<>
struct StampSetter<std_msgs::Header, true>
{
  /**
   * @brief Sets message stamp
   */
  inline void operator()(std_msgs::Header& msg, const ros::Time& stamp) const
  {
    msg.stamp = stamp;
  }
};

}  // namespace flow_ros

#endif  // FLOW_ROS_MESSAGES_STD_MSGS_HEADER_H
