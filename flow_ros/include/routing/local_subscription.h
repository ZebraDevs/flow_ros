/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file  local_subscription.h
 */
#ifndef FLOW_ROS_ROUTING_LOCAL_SUBSCRIPTION_H
#define FLOW_ROS_ROUTING_LOCAL_SUBSCRIPTION_H

// C++ Standard Library
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <thread>
#include <type_traits>

#ifdef FLOW_ROS_HAS_ROSBAG_SUPPORT

// C++ Standard Library
#include <sstream>

// ROS Bag
#include <rosbag/message_instance.h>

#endif  // FLOW_ROS_HAS_ROSBAG_SUPPORT

// Flow
#include <flow_ros/routing/subscription_wrapper.h>

namespace flow_ros
{
namespace routing
{

/**
 * @brief LocalSubscription base which provides basic info and erases MsgT-dependent methods
 */
class LocalSubscriptionBase
{
public:
  virtual ~LocalSubscriptionBase() = default;

  /**
   * @brief Topic setup constructor
   *
   * @param topic  topic name to associate with this object
   */
  explicit LocalSubscriptionBase(std::string topic) :
    topic_{std::move(topic)}
  {}

#ifdef FLOW_ROS_HAS_ROSBAG_SUPPORT

  /**
   * @brief Calls subscriber callback with a rosbag message instance
   */
  virtual void call(const ::rosbag::MessageInstance& mi) const = 0;

#endif  // FLOW_ROS_HAS_ROSBAG_SUPPORT

  /**
   * @brief Returns topic name associated with this object
   */
  inline const std::string& getTopic() const
  {
    return topic_;
  }

private:
  /// Topic name
  std::string topic_;
};


template<typename MsgT>
class LocalSubscription :
  public LocalSubscriptionBase,
  public SubscriptionWrapper
{
public:
  /**
   * @brief Required setup constructor
   *
   * @param topic  topic name to associate with this object
   * @param cb  subscriber callback to associate with this object
   */
  template<typename CallbackT>
  LocalSubscription(std::string topic, CallbackT&& cb) :
    LocalSubscriptionBase{std::move(topic)},
    callback_{std::forward<CallbackT>(cb)}
  {}

  /**
   * @brief Deconstructor
   */
  virtual ~LocalSubscription() = default;

  /**
   * @brief Calls subscriber callback with a new message
   *
   * @param message  message data
   */
  inline void call(const message_shared_const_ptr_t<MsgT>& message) const
  {
    callback_(message);
  }

#ifdef FLOW_ROS_HAS_ROSBAG_SUPPORT

  /**
   * @brief Calls subscriber callback with a rosbag message instance
   *
   * @param mi  ROS bag message instance
   *
   * @throws <code>std::runtime_error</code> on failure to instance message
   * @note participates in overload resolution if <code>MsgT</code> is a ROS message
   */
  void call(const ::rosbag::MessageInstance& mi) const final 
  {
    call_impl(mi);
  }

#endif  // FLOW_ROS_HAS_ROSBAG_SUPPORT

  /**
   * @brief Returns topic name associated with this object
   */
  std::string getTopic() const final
  {
    return LocalSubscriptionBase::getTopic();
  }

  /**
   * @brief Returns number of local publicatiions connected to this subscriber
   */
  std::uint32_t getNumPublishers() const final
  {
    return 0; /*unknown*/
  }

  /**
   * @brief Returns transport method (code) associated with this publisher
   */
  TransportMethod getTransportMethod() const final
  {
    return TransportMethod::LOCAL;
  }

  /**
   * @brief Checks if object is valid
   *
   * @retval true  if a callback has been registered to this object
   * @retval false  otherwise
   */
  bool isValid() const final
  {
    return static_cast<bool>(callback_);
  }

private:

#ifdef FLOW_ROS_HAS_ROSBAG_SUPPORT

  /**
   * @brief Calls subscriber callback with a rosbag message instance
   *
   * @param mi  ROS bag message instance
   *
   * @throws <code>std::runtime_error</code> on failure to instance message
   * @note participates in overload resolution if <code>MsgT</code> is a ROS message
   */
  template<bool U = ros::message_traits::IsMessage<MsgT>::value>
  inline std::enable_if_t<U> call_impl(const ::rosbag::MessageInstance& mi) const 
  {
    const auto msg = mi.template instantiate<std::remove_const_t<MsgT>>();
    if (msg)
    {
      LocalSubscription::call(msg);
    }
    else
    {
      std::ostringstream oss;
      oss << "Could not instance message on topic: " << mi.getTopic() << "(md5sum=" << mi.getMD5Sum() << ')';
      throw std::runtime_error{oss.str()};
    }
  }

  /**
   * @throws <code>std::runtime_error</code> always
   * @note participates in overload resolution if <code>MsgT</code> is NOT a ROS message
   */
  template<bool U = ros::message_traits::IsMessage<MsgT>::value>
  inline std::enable_if_t<!U> call_impl(const ::rosbag::MessageInstance&) const 
  {
    throw std::runtime_error{"Message type is not a ROS message"};
  }

#endif  // FLOW_ROS_HAS_ROSBAG_SUPPORT

  /// Required callback type alias
  using CallbackType = std::function<void(const message_shared_const_ptr_t<MsgT>&)>;

  /// Topic name
  std::string topic_;

  /// Associated subscriber callback
  CallbackType callback_;
};


/**
 * @brief Holds a group of LocalSubscription objects associated with a particular topic
 */
class LocalSubscriptionGroup
{
public:
  /**
   * @brief Calls all subscriber callbacks on a new message
   * @param message  message data
   * @throws <code>std::runtime_error</code> if <code>MsgT</code> is incompatible with subscription group
   * @note Thread safe; injection will be blocked when adding new subscription
   */
  template<typename MsgT>
  inline void call(const message_shared_const_ptr_t<MsgT>& message) const
  {
    // Lock before injection
    std::lock_guard<std::mutex> lock{members_mutex_};

    // All subscriptions are local subscriptions for the same message type
    for (auto it = members_.begin(); it != members_.end(); /*empty*/)
    {
      const auto sub = it->lock();
      if (!static_cast<bool>(sub))
      {
        // Subscriber has expired, remove from list
        it = members_.erase(it);
        continue;
      }
      
      const auto local_sub = std::dynamic_pointer_cast<LocalSubscription<std::add_const_t<MsgT>>>(sub);
      if (static_cast<bool>(local_sub))
      {
        local_sub->call(message);
        ++it;
      }
      else
      {
        throw std::runtime_error{"Invalid message type for subscription"};
      }
    }
  }

#ifdef FLOW_ROS_HAS_ROSBAG_SUPPORT

  /**
   * @brief Calls subscriber callback with a rosbag message instance
   *
   * @param mi  ROS bag message instance
   *
   * @throws <code>std::runtime_error</code> on failure to instance message
   */
  void call(const ::rosbag::MessageInstance& mi) const
  {
    // Lock before injection
    std::lock_guard<std::mutex> lock{members_mutex_};

    for (auto it = members_.begin(); it != members_.end(); /*empty*/)
    {
      const auto sub = it->lock();
      if (static_cast<bool>(sub))
      {
        ++it;
        sub->call(mi); 
      }
      else
      {
        // Subscriber has expired, remove from list
        it = members_.erase(it);
      }
    }
  }

#endif  // FLOW_ROS_HAS_ROSBAG_SUPPORT

  /**
   * @brief Adds new subscriptions to group
   * @param sub  LocalSubscription resource
   * @note Thread safe; injection will be blocked when adding new subscription
   */
  template<typename MsgT>
  inline void addSubscription(std::shared_ptr<LocalSubscription<MsgT>> sub)
  {
    // Lock before adding subscription to block injection
    std::lock_guard<std::mutex> lock{members_mutex_};
    members_.emplace_back(std::move(sub));
  }

  /**
   * @brief Validation cast operator
   *
   * @retval true  if there are any held subscriptions
   * @retval false  otherwise
   */
  inline operator bool() const
  {
    std::lock_guard<std::mutex> lock{members_mutex_};
    return !static_cast<bool>(members_.empty());
  }

  /**
   * @brief Returns the number of held subscriptions
   */
  inline size_t size() const
  {
    std::lock_guard<std::mutex> lock{members_mutex_};
    return members_.size();
  }

private:
  /// Held subscriptions associated with group
  mutable std::vector<std::weak_ptr<LocalSubscriptionBase>> members_;

  /// Mutex which protects member subscriptions
  mutable std::mutex members_mutex_;
};

}  // namespace routing
}  // namespace flow_ros


#endif  // FLOW_ROS_ROUTING_LOCAL_SUBSCRIPTION_H
