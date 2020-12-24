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
   * @brief Returns message stamp, assuming a <code>stamp</code> field exists
   */
  template <typename MsgPtrT> inline static const ros::Time& stamp(const MsgPtrT& message) { return message->stamp; }

  /**
   * @brief Returns reference to message resource pointer (pass-through)
   */
  template <typename MsgPtrT> inline static const MsgPtrT& value(const MsgPtrT& message) { return message; }
};

/**
 * @brief Defines default message accessors to use a fall-back when defining <code>flow::DispatchAccess</code>
 */
struct DefaultMessageHeaderStampDispatchAccess
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
namespace detail
{

/**
 * @brief Extract message value type from message pointer
 */
template <typename MsgT> struct MsgType;
template <typename MsgT> struct MsgType<boost::shared_ptr<const MsgT>>
{
  using type = MsgT;
};
template <typename MsgT> struct MsgType<std::shared_ptr<const MsgT>>
{
  using type = MsgT;
};

/// Helper used to check if <code>MsgT::stamp</code> of type <code>ros::Time</code> exists
template <typename MsgT> struct has_member_stamp
{
  // Dummy template, used to evaluate within argument list
  template <typename C, C> struct TryEval;

  // This method is called if 'MsgT::header' is available
  template <typename C> static constexpr std::true_type test_fn(TryEval<ros::Time MsgT::*, &C::stamp>*);

  // This method called as a fall-back if above "test_fn" is ignore due to SFINAE
  template <typename C> static constexpr std::false_type test_fn(...);

  static constexpr bool value = std::is_same<decltype(test_fn<MsgT>(nullptr)), std::true_type>::value;
};

/// Helper used to check if <code>MsgT::header</code> of type <code>std_msgs::Header</code> exists
template <typename MsgT> struct has_member_header
{
  // Dummy template, used to evaluate within argument list
  template <typename C, C> struct TryEval;

  // This method is called if 'MsgT::header' is available
  template <typename C> static constexpr std::true_type test_fn(TryEval<std_msgs::Header MsgT::*, &C::header>*);

  // This method called as a fall-back if above "test_fn" is ignore due to SFINAE
  template <typename C> static constexpr std::false_type test_fn(...);

  static constexpr bool value = std::is_same<decltype(test_fn<MsgT>(nullptr)), std::true_type>::value;
};

}  // namespace detail

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
template <typename MsgT>
struct DispatchAccess : std::conditional_t<
                          detail::has_member_stamp<typename detail::MsgType<MsgT>::type>::value,
                          ::flow_ros::DefaultMessageStampDispatchAccess,
                          ::flow_ros::DefaultMessageHeaderStampDispatchAccess>
{
  FLOW_STATIC_ASSERT(
    !(detail::has_member_stamp<typename detail::MsgType<MsgT>::type>::value and
      detail::has_member_header<typename detail::MsgType<MsgT>::type>::value),
    "Default access implementation detects both 'ros::Time stamp' and 'std_msgs::Header header' members. "
    "Please specialize 'DispatchAccess' for your message type to disambiguate.");
};

}  // namespace flow

#endif  // FLOW_ROS_MESSAGE_STAMP_ACCESS_H
