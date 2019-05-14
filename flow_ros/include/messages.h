/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 * 
 * @file  messages.h
 */
#ifndef FLOW_ROS_MESSAGES_H
#define FLOW_ROS_MESSAGES_H

// C++ Standard Library
#include <memory>
#include <vector>
#include <type_traits>

// ROS
#include <ros/time.h>
#include <ros/message_traits.h>

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



/// Extracts appropriate shared_ptr wrapper type for <code>MsgT</code>
template <typename MsgT>
using message_shared_ptr_t = typename MessageSharedPtrType<MsgT>::type;


/// Extracts appropriate shared_ptr wrapper type for <code>MsgT</code>
template <typename MsgT>
using message_shared_const_ptr_t = typename MessageSharedConstPtrType<MsgT>::type;


/**
 * @brief Flow dispatch object for ROS messages with a Header field
 *
 * @tparam MsgT  message type
 * @tparam IS_ROS_MESSAGE  [DO NOT SUPPLY THIS ARGUMENT] (defaulted) to see if MsgT is a ROS message
 *
 * @note May be specialized for any message which time stamp info
 */
template<typename MsgT, bool IS_ROS_MESSAGE = is_message_v<MsgT>>
class MessageDispatch;


/**
 * @brief Helper object use to set message stamps
 *
 * @tparam MsgT  message type
 * @tparam IS_ROS_MESSAGE  [DO NOT SUPPLY THIS ARGUMENT] (defaulted) to see if MsgT is a ROS message
 */
template<typename MsgT, bool IS_ROS_MESSAGE = is_message_v<MsgT>>
struct StampSetter;


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
 * @brief Helper which extracts message data from dispatches
 * @tparam MsgT  (deduced) message type
 * @tparam AllocatorT  (deduced) allocator type
 */
template<typename MsgT, typename  AllocatorT>
inline std::vector<message_shared_const_ptr_t<MsgT>> extract_messages(const std::vector<MessageDispatch<MsgT>, AllocatorT>& dispatches)
{
  std::vector<message_shared_const_ptr_t<MsgT>> messages;
  messages.reserve(dispatches.size());
  std::transform(dispatches.begin(),
                 dispatches.end(),
                 std::back_inserter(messages),
                 [](const MessageDispatch<MsgT>& dispatch)
                 {
                   return dispatch.data();
                 });
  return messages;
}


/**
 * @brief Helper which extracts time stamp data from dispatches
 * @tparam MsgT  (deduced) time stamp type
 * @tparam AllocatorT  (deduced) allocator type
 */
template<typename MsgT, typename  AllocatorT>
inline std::vector<ros::Time> extract_stamps(const std::vector<MessageDispatch<MsgT>, AllocatorT>& dispatches)
{
  std::vector<ros::Time> stamps;
  stamps.reserve(dispatches.size());
  std::transform(dispatches.begin(),
                 dispatches.end(),
                 std::back_inserter(stamps),
                 [](const MessageDispatch<MsgT>& dispatch)
                 {
                   return dispatch.stamp();
                 });
  return stamps;
}


/**
 * @brief Helper which extracts first message data from dispatches
 * @tparam MsgT  (deduced) message type
 * @tparam AllocatorT  (deduced) allocator type
 */
template<typename MsgT, typename  AllocatorT>
inline message_shared_const_ptr_t<MsgT> extract_first_message(const std::vector<MessageDispatch<MsgT>, AllocatorT>& dispatches)
{
  return dispatches.front().data();
}


/**
 * @brief Helper which extracts last message data from dispatches
 * @tparam MsgT  (deduced) message type
 * @tparam AllocatorT  (deduced) allocator type
 */
template<typename MsgT, typename  AllocatorT>
inline message_shared_const_ptr_t<MsgT> extract_last_message(const std::vector<MessageDispatch<MsgT>, AllocatorT>& dispatches)
{
  return dispatches.back().data();
}


/**
 * @brief Helper which extracts first time stamp data from dispatches
 * @tparam MsgT  (deduced) time stamp type
 * @tparam AllocatorT  (deduced) allocator type
 */
template<typename MsgT, typename  AllocatorT>
inline typename ros::Time extract_first_stamp(const std::vector<MessageDispatch<MsgT>, AllocatorT>& dispatches)
{
  return dispatches.front().stamp();
}


/**
 * @brief Helper which extracts last time stamp data from dispatches
 * @tparam MsgT  (deduced) time stamp type
 * @tparam AllocatorT  (deduced) allocator type
 */
template<typename MsgT, typename  AllocatorT>
inline typename ros::Time extract_last_stamp(const std::vector<MessageDispatch<MsgT>, AllocatorT>& dispatches)
{
  return dispatches.back().stamp();
}

}  // namespace flow_ros

// Flow (implementation)
#include <flow_ros/impl/messages.hpp>

#endif  // FLOW_ROS_MESSAGES_H
