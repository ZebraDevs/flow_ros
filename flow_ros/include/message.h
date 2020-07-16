/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 * 
 * @file  message.h
 */
#ifndef FLOW_ROS_MESSAGE_H
#define FLOW_ROS_MESSAGE_H

// C++ Standard Library
#include <memory>
#include <vector>
#include <type_traits>

// Boost (ROS message Ptr/ConstPtr)
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/time.h>
#include <ros/message_traits.h>

// Flow
#include <flow/dispatch.h>

namespace flow_ros
{

/// Checks if object is a ROS message
template <typename MsgT>
constexpr bool is_message_v = ros::message_traits::IsMessage<std::remove_const_t<MsgT>>::value;

/**
 * @brief Traits object used to select correct shared_ptr wrapper type
 *
 * @tparam MsgT  message type
 * @tparam IS_ROS_MESSAGE  [DO NOT SUPPLY THIS ARGUMENT] (defaulted) to see if MsgT is a ROS message
 */
template<typename MsgT, bool IS_ROS_MESSAGE = is_message_v<MsgT>>
struct MessageSharedPtrType
{
  using type = std::shared_ptr<MsgT>;
};


/**
 * @brief Traits object used to select correct shared_ptr const value type wrapper
 *
 * @tparam MsgT  message type
 * @tparam IS_ROS_MESSAGE  [DO NOT SUPPLY THIS ARGUMENT] (defaulted) to see if MsgT is a ROS message
 */
template<typename MsgT, bool IS_ROS_MESSAGE = is_message_v<MsgT>>
struct MessageSharedConstPtrType
{
  using type = std::shared_ptr<const std::remove_const_t<MsgT>>;
};


/**
 * @copydoc MessageSharedPtrType
 * @note Specialization for ROS messages
 */
template<typename MsgT>
struct MessageSharedPtrType<MsgT, true>
{
  using type = boost::shared_ptr<MsgT>;
};


/**
 * @copydoc MessageSharedConstPtrType
 * @note Specialization for ROS messages
 */
template<typename MsgT>
struct MessageSharedConstPtrType<MsgT, true>
{
  using type = boost::shared_ptr<const std::remove_const_t<MsgT>>;
};


/// Extracts appropriate shared_ptr wrapper type for <code>MsgT</code>
template <typename MsgT>
using message_shared_ptr_t = typename MessageSharedPtrType<MsgT>::type;


/// Extracts appropriate shared_ptr wrapper type for <code>MsgT</code>
template <typename MsgT>
using message_shared_const_ptr_t = typename MessageSharedConstPtrType<MsgT>::type;


/**
 * @brief Helper object use to set message stamps
 *
 * @tparam MsgT  message type
 *
 * @note May be specialized for any message which time stamp info
 */
template<typename MsgT>
struct StampSetter
{
  inline void operator()(MsgT& msg, const ros::Time& stamp) const
  {
    msg.header.stamp = stamp;
  }
};


/**
 * @brief Helper function use to set message stamp with StampSetter
 *
 * @tparam MsgT  (deduced) message type
 *
 * @param msg  message
 * @param stamp  stamp to set
 */
template<typename MsgT>
inline void set_stamp(MsgT&& msg, const ros::Time& stamp)
{
  StampSetter<MsgT>{}(std::forward<MsgT>(msg), stamp);
}

/**
 * @brief Defines default message accessors to use a fall-back when defining <code>flow::DispatchAccess</code>
 */
struct DefaultMessageDispatchAccess
{
  /**
   * @brief Returns message stamp, assuming a <code>header.stamp</code> field exists
   */
  template <typename MsgPtrT>
  inline static const ros::Time& stamp(const MsgPtrT& message) { return message->header.stamp; }

  /**
   * @brief Returns reference to message resource pointer (pass-through)
   */
  template <typename MsgPtrT>
  inline static const MsgPtrT& value(const MsgPtrT& message) { return message; }
};

}  // namespace flow_ros


namespace flow
{

/**
 * @brief ROS timing type traits for associated message Dispatch
 */
template<>
struct StampTraits<ros::Time>
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
template<typename MsgT>
struct DispatchTraits<boost::shared_ptr<const MsgT>>
{
  /// Dispatch stamp type
  using stamp_type = ros::Time;

  /// Dispatch data type
  using value_type = boost::shared_ptr<const MsgT>;
};


/**
 * @brief Default ROS message-like Dispatch traits
 */
template<typename MsgT>
struct DispatchTraits<std::shared_ptr<const MsgT>>
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
template<typename MsgT>
struct DispatchAccess : ::flow_ros::DefaultMessageDispatchAccess {};

}  // namespace flow

#endif  // FLOW_ROS_MESSAGE_H
