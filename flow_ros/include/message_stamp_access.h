/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file  message_stamp_access.h
 */
#ifndef FLOW_ROS_MESSAGE_STAMP_ACCESS_H
#define FLOW_ROS_MESSAGE_STAMP_ACCESS_H

#ifdef FLOW_ROS_MESSAGE_SEQ_ACCESS_H
#error Either "message_seq_access.h" or "message_stamp_access.h" can be included, but not both
#endif  // FLOW_ROS_MESSAGE_SEQ_ACCESS_H

// C++ Standard Library
#include <memory>
#include <type_traits>
#include <vector>

// Boost (ROS message Ptr/ConstPtr)
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/time.h>

// Flow
#include <flow/dispatch.h>
#include <flow_ros/message_ptr.h>

namespace flow_ros
{

/**
 * @brief Helper object use to set message stamps
 *
 * @tparam MsgT  message type
 *
 * @note May be specialized for any message with a stamp
 */
template <typename MsgT> struct StampSetter
{
  inline void operator()(MsgT& msg, const ros::Time& stamp) const { msg.header.stamp = stamp; }
};


/**
 * @brief Helper function use to set message stamp with StampSetter
 *
 * @tparam MsgT  (deduced) message type
 *
 * @param msg  message
 * @param stamp  stamp to set
 */
template <typename MsgT> inline void set_stamp(MsgT&& msg, const ros::Time& stamp)
{
  StampSetter<MsgT>{}(std::forward<MsgT>(msg), stamp);
}

/**
 * @brief Defines default message accessors to use a fall-back when defining <code>flow::DispatchAccess</code>
 */
struct DefaultMessageStampDispatchAccess
{
  /**
   * @brief Returns message stamp, assuming a <code>header.stamp</code> field exists
   */
  template <typename MsgPtrT> inline static const ros::Time& stamp(const MsgPtrT& message)
  {
    return message->header.stamp;
  }

  /**
   * @brief Returns reference to message resource pointer (pass-through)
   */
  template <typename MsgPtrT> inline static const MsgPtrT& value(const MsgPtrT& message) { return message; }
};

}  // namespace flow_ros


namespace flow
{

/**
 * @brief ROS timing type traits for associated message Dispatch
 */
template <> struct StampTraits<ros::Time>
{
  /// Stamp type
  using stamp_type = ros::Time;

  /// Associated duration/offset type
  using offset_type = ros::Duration;

  /// Returns minimum stamp value
  static ros::Time min() { return ros::Time{1}; };

  /// Returns maximum stamp value
  static ros::Time max() { return ros::TIME_MAX; };
};


/**
 * @brief Default ROS message Dispatch traits
 */
template <typename MsgT> struct DispatchTraits<boost::shared_ptr<const MsgT>>
{
  /// Dispatch stamp type
  using stamp_type = ros::Time;

  /// Dispatch data type
  using value_type = boost::shared_ptr<const MsgT>;
};


/**
 * @brief Default ROS message-like Dispatch traits
 */
template <typename MsgT> struct DispatchTraits<std::shared_ptr<const MsgT>>
{
  /// Dispatch stamp type
  using stamp_type = ros::Time;

  /// Dispatch data type
  using value_type = std::shared_ptr<const MsgT>;
};


/**
 * @brief Template which basic methods for MessageDispatch
 *
 * @tparam MsgT  message type
 */
template <typename MsgT> struct DispatchAccess : ::flow_ros::DefaultMessageStampDispatchAccess
{};

}  // namespace flow

#endif  // FLOW_ROS_MESSAGE_STAMP_ACCESS_H
