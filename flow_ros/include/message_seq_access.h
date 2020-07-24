/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file  message_seq_access.h
 */
#ifndef FLOW_ROS_MESSAGE_SEQ_ACCESS_H
#define FLOW_ROS_MESSAGE_SEQ_ACCESS_H

#ifdef FLOW_ROS_MESSAGE_STAMP_ACCESS_H
#error Either "message_seq_access.h" or "message_stamp_access.h" can be included, but not both
#endif  // FLOW_ROS_MESSAGE_STAMP_ACCESS_H

// C++ Standard Library
#include <cstdint>
#include <memory>
#include <type_traits>
#include <vector>

// Boost (ROS message Ptr/ConstPtr)
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/message_traits.h>
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
 * @note May be specialized for any message with a sequence count
 */
template <typename MsgT> struct StampSetter
{
  inline void operator()(MsgT& msg, std::uint32_t seq) const { msg.header.seq = seq; }
};


/**
 * @brief Defines default message accessors to use a fall-back when defining <code>flow::DispatchAccess</code>
 */
struct DefaultMessageSeqDispatchAccess
{
  /**
   * @brief Returns message stamp, assuming a <code>header.stamp</code> field exists
   */
  template <typename MsgPtrT> inline static std::uint32_t stamp(const MsgPtrT& message)
  {
    return message->header.seq;
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
 * @brief Default ROS message Dispatch traits
 */
template <typename MsgT> struct DispatchTraits<boost::shared_ptr<const MsgT>>
{
  /// Dispatch stamp type
  using stamp_type = std::uint32_t;

  /// Dispatch data type
  using value_type = boost::shared_ptr<const MsgT>;
};


/**
 * @brief Default ROS message-like Dispatch traits
 */
template <typename MsgT> struct DispatchTraits<std::shared_ptr<const MsgT>>
{
  /// Dispatch stamp type
  using stamp_type = std::uint32_t;

  /// Dispatch data type
  using value_type = std::shared_ptr<const MsgT>;
};


/**
 * @brief Template which basic methods for MessageDispatch
 *
 * @tparam MsgT  message type
 */
template <typename MsgT> struct DispatchAccess : ::flow_ros::DefaultMessageSeqDispatchAccess
{};

}  // namespace flow

#endif  // FLOW_ROS_MESSAGE_SEQ_ACCESS_H
