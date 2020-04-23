/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_ROS_IMPL_MESSAGES_HPP
#define FLOW_ROS_IMPL_MESSAGES_HPP

// C++ Standard Library
#include <type_traits>
#include <utility>

// Boost (ROS message Ptr/ConstPtr)
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/time.h>
#include <ros/message_traits.h>

namespace flow_ros
{

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


// Specialization for message-like objects (to be passed locally) with a <code>header</code> field
template<typename MsgT>
class MessageDispatch<MsgT, false>
{
public:
  MessageDispatch() = default;
  MessageDispatch(const MessageDispatch&) = default;

  /**
   * @brief Message constructor (move enabled)
   */
  explicit MessageDispatch(typename MessageSharedPtrType<MsgT>::type msg) :
    msg_{std::move(msg)}
  {}

  /**
   * @brief Returns sequencing stamp associated with data element
   */
  inline const ros::Time& stamp() const
  {
    return msg_->header.stamp;
  }

  /**
   * @brief Returns const reference to underlying data element
   */
  inline const typename MessageSharedPtrType<MsgT>::type& data() const
  {
    return msg_;
  }

  /**
   * @brief Derefences message resource
   */
  inline const auto& operator*() const
  {
    return *msg_;
  }

  /**
   * @brief Derefences message resource
   */
  inline const auto* operator->() const
  {
    return std::addressof(*msg_);
  }

  /**
   * @brief Checks if message contents are valid
   */
  inline bool valid() const
  {
    return static_cast<bool>(msg_);
  }

  /**
   * @copydoc MessageDispatch::valid
   */
  inline operator bool() const
  {
    return this->valid();
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
  typename MessageSharedPtrType<MsgT>::type msg_;
};


// Specialization for ROS messages with <code>header</code> fields
template<typename MsgT>
struct MessageDispatch<MsgT, true> : MessageDispatch<MsgT, false>
{
  static_assert(ros::message_traits::HasHeader<MsgT>::value,
                "'MsgT::header' is not an available field");

  MessageDispatch() = default;
  MessageDispatch(const MessageDispatch&) = default;

  /**
   * @brief Message constructor (move enabled)
   */
  explicit MessageDispatch(typename MessageSharedPtrType<MsgT>::type msg) :
    MessageDispatch<MsgT, false>{std::move(msg)}
  {}
};


// Specialization for message-like objects (to be passed locally) with a <code>header</code> field
template<typename MsgT>
struct StampSetter<MsgT, false>
{
  /**
   * @brief Sets message stamp
   */
  inline void operator()(MsgT& msg, const ros::Time& stamp) const
  {
    msg.header.stamp = stamp;
  }
};


// Specialization for ROS messages with <code>header</code> fields
template<typename MsgT>
struct StampSetter<MsgT, true> : StampSetter<MsgT, false>
{
  static_assert(ros::message_traits::HasHeader<MsgT>::value,
                "'MsgT::header' is not an available field");
};

}  // namespace flow_ros

#endif  // FLOW_ROS_IMPL_MESSAGES_HPP
