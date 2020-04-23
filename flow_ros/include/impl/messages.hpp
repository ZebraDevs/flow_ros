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

}  // namespace flow_ros

#endif  // FLOW_ROS_IMPL_MESSAGES_HPP
