/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file  message_ptr.h
 */
#ifndef FLOW_ROS_MESSAGE_PTR_H
#define FLOW_ROS_MESSAGE_PTR_H

// C++ Standard Library
#include <type_traits>

// Boost (ROS message Ptr/ConstPtr)
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/message_traits.h>

/**
 * ROS messages are stored as <code>boost::shared_ptr</code> resources, which was
 * a design decision made before C++ had introduce <code>std::shared_ptr</code> with
 * C++11.
 *
 * This definition provides a fall-back for non-message entities passed between
 * flow_ros Publisher and Subscriber objects through local message passing
 * using flow_ros::Router, since there is no restriction to use boost in these cases.
 *
 * If you would like to use <code>boost::shared_ptr</code>, or something else, define
 * <code>NON_ROS_MESSAGE_SHARED_PTR_TMPL = *shared_ptr*</code> before this point
 */
#ifndef NON_ROS_MESSAGE_SHARED_PTR_TMPL

#include <memory>

#define NON_ROS_MESSAGE_SHARED_PTR_TMPL std::shared_ptr<MsgT>

#endif  // NON_ROS_MESSAGE_SHARED_PTR_TMPL


namespace flow_ros
{

/// Checks if object is a ROS message
template <typename MsgT> constexpr bool is_message_v = ros::message_traits::IsMessage<std::remove_const_t<MsgT>>::value;


/// Non-message resource pointer alias
template <typename MsgT>
using non_message_shared_ptr_t = NON_ROS_MESSAGE_SHARED_PTR_TMPL;


/**
 * @brief Traits object used to select correct shared_ptr resource type
 *
 * @tparam MsgT  message type
 * @tparam IS_ROS_MESSAGE  [DO NOT SUPPLY THIS ARGUMENT] (defaulted) to see if MsgT is a ROS message
 */
template <typename MsgT, bool IS_ROS_MESSAGE = is_message_v<MsgT>> struct MessageSharedPtrType
{
  using type = non_message_shared_ptr_t<MsgT>;
};


/**
 * @brief Traits object used to select correct shared_ptr const resource wrapper
 *
 * @tparam MsgT  message type
 * @tparam IS_ROS_MESSAGE  [DO NOT SUPPLY THIS ARGUMENT] (defaulted) to see if MsgT is a ROS message
 */
template <typename MsgT, bool IS_ROS_MESSAGE = is_message_v<MsgT>> struct MessageSharedConstPtrType
{
  using type = non_message_shared_ptr_t<const std::remove_const_t<MsgT>>;
};


/**
 * @copydoc MessageSharedPtrType
 * @note Specialization for ROS messages
 */
template <typename MsgT> struct MessageSharedPtrType<MsgT, true>
{
  using type = boost::shared_ptr<MsgT>;
};


/**
 * @copydoc MessageSharedConstPtrType
 * @note Specialization for ROS messages
 */
template <typename MsgT> struct MessageSharedConstPtrType<MsgT, true>
{
  using type = boost::shared_ptr<const std::remove_const_t<MsgT>>;
};


/// Extracts appropriate shared_ptr wrapper type for <code>MsgT</code>
template <typename MsgT> using message_shared_ptr_t = typename MessageSharedPtrType<MsgT>::type;


/// Extracts appropriate shared_ptr wrapper type for <code>MsgT</code>
template <typename MsgT> using message_shared_const_ptr_t = typename MessageSharedConstPtrType<MsgT>::type;


}  // namespace flow_ros

#endif  // FLOW_ROS_MESSAGE_PTR_H
