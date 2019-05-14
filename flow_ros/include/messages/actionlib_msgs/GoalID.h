/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 * 
 * @file GoalID.h
 */
#ifndef FLOW_ROS_MESSAGES_ACTIONLIB_MSGS_GOALID_H
#define FLOW_ROS_MESSAGES_ACTIONLIB_MSGS_GOALID_H

// ROS
#include <ros/time.h>

// Messages
#include <actionlib_msgs/GoalID.h>

// Flow
#include <flow_ros/messages.h>

namespace flow_ros
{

// actionlib_msgs/GoalID specialization of MessageDispatch
template<>
class MessageDispatch<const actionlib_msgs::GoalID, true>
{
public:
  MessageDispatch() = default;
  MessageDispatch(const MessageDispatch&) = default;

  /**
   * @brief Message constructor (move enabled)
   */
  explicit MessageDispatch(typename actionlib_msgs::GoalID::ConstPtr msg) :
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
  inline const typename actionlib_msgs::GoalID::ConstPtr& data() const
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
  typename actionlib_msgs::GoalID::ConstPtr msg_;
};


// actionlib_msgs/GoalID specialization of StampSetter
template<>
struct StampSetter<actionlib_msgs::GoalID, true>
{
  /**
   * @brief Sets message stamp
   */
  inline void operator()(actionlib_msgs::GoalID& msg, const ros::Time& stamp) const
  {
    msg.stamp = stamp;
  }
};

}  // namespace flow_ros

#endif  // FLOW_ROS_MESSAGES_ACTIONLIB_MSGS_GOALID_H
