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
 * @brief Template which basic methods for MessageDispatch
 *
 * @tparam MsgT  message type
 */
template<typename MsgT>
class MessageDispatchSpecializationBase
{
public:
  using message_resource_type = typename MessageSharedPtrType<MsgT>::type;

  MessageDispatchSpecializationBase() = default;
  MessageDispatchSpecializationBase(const MessageDispatchSpecializationBase&) = default;

  /**
   * @brief Message constructor (move enabled)
   */
  explicit MessageDispatchSpecializationBase(message_resource_type msg) :
    msg_{std::move(msg)}
  {}

  /**
   * @brief Checks if message contents are valid
   */
  inline bool valid() const
  {
    return static_cast<bool>(msg_);
  }

  /**
   * @copydoc MessageDispatchSpecializationBase::valid
   */
  inline operator bool() const
  {
    return this->valid();
  }

  /**
   * @brief Derefences message resource
   */
  inline const MsgT& operator*() const
  {
    return *msg_;
  }

  /**
   * @brief Derefences message resource
   */
  inline const MsgT* operator->() const
  {
    return msg_.get();
  }

  /**
   * @brief Message resource conversion operator overload
   */
  inline operator const message_resource_type&() const
  {
    return msg_;
  }

  /**
   * @brief Returns const reference to underlying data element
   */
  inline const typename MessageSharedPtrType<MsgT>::type& data() const
  {
    return this->msg_;
  }

protected:
  /// Message payload
  typename MessageSharedPtrType<MsgT>::type msg_;
};


/**
 * @brief Flow dispatch object for ROS messages with a Header field
 *
 * @tparam MsgT  message type
 *
 * @note May be specialized for any message which time stamp info
 */
template<typename MsgT>
class MessageDispatch : public MessageDispatchSpecializationBase<MsgT>
{
public:
  template<typename ConstructorMsgT>
  inline MessageDispatch(ConstructorMsgT&& msg) :
    MessageDispatchSpecializationBase<MsgT>{std::forward<ConstructorMsgT>(msg)}
  {}

  /**
   * @brief Returns sequencing stamp associated with data element
   */
  inline const ros::Time& stamp() const
  {
    return this->msg_->header.stamp;
  }
};


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

}  // namespace flow_ros

// Flow (implementation)
#include <flow_ros/impl/messages.hpp>

#endif  // FLOW_ROS_MESSAGES_H
