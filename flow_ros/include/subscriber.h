/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file  subscriber.h
 */
#ifndef FLOW_ROS_SUBSCRIBER_H
#define FLOW_ROS_SUBSCRIBER_H

// C++ Standard Library
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

// ROS
#include <ros/message_traits.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

// Flow
#include <flow/drivers.h>
#include <flow/followers.h>
#include <flow_ros/message_ptr.h>
#include <flow_ros/router.h>
#include <flow_ros/routing/local_subscription.h>
#include <flow_ros/routing/ros_subscription.h>
#include <flow_ros/routing/subscription_wrapper.h>

namespace flow_ros
{

/**
 * @brief Message input channel with meta information interfaces
 */
class SubscriberBase
{
public:
  /**
   * @copydoc routing::SubscriptionWrapper::getTopic
   */
  inline std::string getTopic() const { return subscription_->getTopic(); }

  /**
   * @copydoc routing::SubscriptionWrapper::getNumPublishers
   */
  inline std::uint32_t getNumPublishers() const { return subscription_->getNumPublishers(); }

  /**
   * @copydoc routing::SubscriptionWrapper::getTransportMethod
   */
  inline routing::TransportMethod getTransportMethod() const { return subscription_->getTransportMethod(); }

  /**
   * @copydoc routing::SubscriptionWrapper::getTransportDirection
   */
  static constexpr routing::Direction getTransportDirection()
  {
    return routing::SubscriptionWrapper::getTransportDirection();
  }

  /**
   * @brief Returns underlying <code>flow::Captor</code> buffer capacity
   */
  virtual std::size_t getCapacity() const = 0;

  /**
   * @brief Returns number of buffers messages in underlying <code>flow::Captor</code> buffer
   */
  virtual std::size_t getNumBuffered() const = 0;

  /**
   * @brief Returns stamp range of all buffered messages
   */
  virtual flow::CaptureRange<ros::Time> getAvailableBufferedRange() const = 0;

  /**
   * @copydoc routing::SubscriptionWrapper::isValid
   */
  inline bool isValid() const { return subscription_ and subscription_->isValid(); }

  /**
   * @brief Returns underlying subscription implementation wrapper resource
   */
  inline std::shared_ptr<routing::SubscriptionWrapper> impl() const { return subscription_; }

protected:
  /// Underling subscription wrapper
  std::shared_ptr<routing::SubscriptionWrapper> subscription_;
};


/**
 * @brief Message input channel with an associated capture policy
 *
 * @tparam PolicyT  underlying Captor type with synchronization policy
 */
template <typename PolicyT> class SubscriberPolicyBase : public PolicyT, public SubscriberBase
{
public:
  /// Check that PolicyT is a flow::Captor derivative
  static_assert(
    flow::is_driver<PolicyT>() or flow::is_follower<PolicyT>(),
    "'PolicyT' must derived from 'flow::Captor' (must be a Driver or Follower derivative)");

  /**
   * @brief Returns underlying <code>flow::Captor</code> buffer capacity
   */
  std::size_t getCapacity() const final { return PolicyT::get_capacity(); }

  /**
   * @brief Returns number of buffers messages in underlying <code>flow::Captor</code> buffer
   */
  std::size_t getNumBuffered() const final { return PolicyT::size(); }

  /**
   * @brief Returns stamp range of all buffered messages
   */
  flow::CaptureRange<ros::Time> getAvailableBufferedRange() const final { return PolicyT::get_available_stamp_range(); }

protected:
  using SubscriberBase::subscription_;

  /**
   * @brief Policy setup constructor
   */
  template <typename... PolicyArgTs>
  SubscriberPolicyBase(PolicyArgTs&&... policy_args) : PolicyT{std::forward<PolicyArgTs>(policy_args)...}
  {}
};


/**
 * @brief Input channel which subscribes to messages
 *
 *        This input channel object wraps an underlying message transport subscription
 *        and a synchronization policy which works on buffered data
 *
 * @tparam MsgT  message type
 * @tparam PolicyTmpl  name associated with policy to be used when capturing inputs
 * @tparam LockPolicyT  a BasicLockable (https://en.cppreference.com/w/cpp/named_req/BasicLockable) object;
 *                      using <code>flow::NoLock</code> will bypass internal thread locking/safety mechanisms
 * @tparam MsgConstPtrContainerT  underlying container type used to store shared message resource pointers
 */
template <
  typename MsgT,
  template <typename...>
  class PolicyTmpl,
  typename LockPolicyT = std::unique_lock<std::mutex>,
  typename MsgConstPtrContainerT = flow::DefaultContainer<message_shared_const_ptr_t<const MsgT>>>
class Subscriber final : public SubscriberPolicyBase<
                           PolicyTmpl<message_shared_const_ptr_t<const MsgT>, LockPolicyT, MsgConstPtrContainerT>>
{
  /// Subscriber base type alias
  using PolicyType =
    SubscriberPolicyBase<PolicyTmpl<message_shared_const_ptr_t<const MsgT>, LockPolicyT, MsgConstPtrContainerT>>;

public:
  /**
   * @brief <code>ros::NodeHandle</code> setup constructor (no transport hints)
   *
   * @tparam PolicyArgTs  type associated with arguments passed to underlying captor type
   *
   * @param[in,out] nh  ROS node handle to negotiate topic subscription/connection
   * @param topic  topic associated with input messages
   * @param queue_size  underlying ROS subscriber queue size
   * @param args  arguments to pass to underlying captor types as
   *              <code>PolicyTmpl(args,...)</code>
   *
   * @warning Will throw with <code>std::runtime_error</code> if underlying subscriber
   *          could not be establish a connection with ROS-master
   */
  template <typename... PolicyArgTs>
  Subscriber(ros::NodeHandle& nh, const std::string& topic, const std::uint32_t queue_size, PolicyArgTs&&... args) :
      PolicyType{std::forward<PolicyArgTs>(args)...}
  {
    // Subscription must be setup after buffer so that object is initialized before callbacks run
    SubscriberBase::subscription_ =
      std::make_shared<routing::ROSSubscription>(nh.subscribe(topic, queue_size, &Subscriber::inject, this));
  }

  /**
   * @brief <code>ros::NodeHandle</code> setup constructor (with transport hints)
   *
   * @tparam PolicyArgTs  type associated with arguments passed to underlying captor type
   *
   * @param[in,out] nh  ROS node handle to negotiate topic subscription/connection
   * @param topic  topic associated with input messages
   * @param transport_hints  a <code>ros::TransportHints</code> structure which defines various transport-related
   * options
   * @param queue_size  underlying ROS subscriber queue size
   * @param args  arguments to pass to underlying captor types as
   *              <code>PolicyTmpl(args,...)</code>
   *
   * @warning Will throw with <code>std::runtime_error</code> if underlying subscriber
   *          could not be establish a connection with ROS-master
   */
  template <typename... PolicyArgTs>
  Subscriber(
    ros::NodeHandle& nh,
    const std::string& topic,
    const ros::TransportHints& transport_hints,
    const std::uint32_t queue_size,
    PolicyArgTs&&... args) :
      PolicyType{std::forward<PolicyArgTs>(args)...}
  {
    // Subscription must be setup after buffer so that object is initialized before callbacks run
    SubscriberBase::subscription_ = std::make_shared<routing::ROSSubscription>(
      nh.subscribe(topic, queue_size, &Subscriber::inject, this, transport_hints));
  }

  /**
   * @brief <code>Router</code> setup constructor
   *
   * @tparam PolicyArgTs  type associated with arguments passed to underlying captor type
   *
   * @param[in,out] router  Intra-node Router routing object
   * @param topic  topic associated with input messages
   * @param queue_size  underlying subscription queue size
   * @param args  arguments to pass to underlying captor types as
   *              <code>PolicyTmpl(args,...)</code>
   */
  template <typename... PolicyArgTs>
  Subscriber(Router& router, std::string topic, const std::uint32_t queue_size, PolicyArgTs&&... args) :
      PolicyType{std::forward<PolicyArgTs>(args)...}
  {
    // Subscription must be setup after buffer so that object is initialized before callbacks run
    SubscriberBase::subscription_ = router.subscribe<MsgT>(std::move(topic), queue_size, &Subscriber::inject, this);
  }

  /**
   * @brief Injects message into captor; automatically
   */
  inline void inject(message_shared_const_ptr_t<MsgT> msg) { PolicyType::inject(std::move(msg)); }
};


/**
 * @brief Subscriber type traits
 */
template <typename MsgT> struct SubscriberTraits;


/**
 * @copydoc SubscriberTraits
 * @note Subscriber partial specialization
 */
template <typename MsgT, template <typename...> class PolicyTmpl, typename LockPolicyT>
struct SubscriberTraits<Subscriber<MsgT, PolicyTmpl, LockPolicyT>>
{
  /// Input message type
  using MsgType = MsgT;

  /// Base capture policy type
  using PolicyType = PolicyTmpl<message_shared_const_ptr_t<MsgT>, LockPolicyT>;
};

}  // namespace flow_ros


namespace flow
{

/**
 * @brief Checks if subscriber capture policy is derived from a Driver base
 */
template <typename MsgT, template <typename...> class PolicyTmpl, typename LockPolicyT>
struct is_driver<::flow_ros::Subscriber<MsgT, PolicyTmpl, LockPolicyT>>
    : is_driver<PolicyTmpl<::flow_ros::message_shared_const_ptr_t<MsgT>, LockPolicyT>>
{};


/**
 * @brief Checks if subscriber capture policy is derived from a Follower base
 */
template <typename MsgT, template <typename...> class PolicyTmpl, typename LockPolicyT>
struct is_follower<::flow_ros::Subscriber<MsgT, PolicyTmpl, LockPolicyT>>
    : is_follower<PolicyTmpl<::flow_ros::message_shared_const_ptr_t<MsgT>, LockPolicyT>>
{};


/**
 * @brief Captor traits associated with underlying Captor of SubscriberPolicyBase
 */
template <typename PolicyT> struct CaptorTraits<::flow_ros::SubscriberPolicyBase<PolicyT>> : CaptorTraits<PolicyT>
{};


/**
 * @brief Captor traits associated with underlying Captor of Subscriber
 */
template <typename MsgT, template <typename...> class PolicyTmpl, typename LockPolicyT>
struct CaptorTraits<::flow_ros::Subscriber<MsgT, PolicyTmpl, LockPolicyT>>
    : CaptorTraits<PolicyTmpl<::flow_ros::message_shared_const_ptr_t<MsgT>, LockPolicyT>>
{};

}  // namespace flow

#endif  // FLOW_ROS_SUBSCRIBER_H
