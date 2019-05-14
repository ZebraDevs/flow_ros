/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 * 
 * @file  router.h
 */
#ifndef FLOW_ROS_ROUTER_H
#define FLOW_ROS_ROUTER_H

// C++ Standard Library
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>

#ifdef FLOW_ROS_HAS_ROSBAG_SUPPORT

// ROS Bag
#include <rosbag/message_instance.h>

#endif  // FLOW_ROS_HAS_ROSBAG_SUPPORT

// Flow
#include <flow_ros/messages.h>
#include <flow_ros/routing/publication_wrapper.h>
#include <flow_ros/routing/subscription_wrapper.h>
#include <flow_ros/routing/local_publication.h>
#include <flow_ros/routing/local_subscription.h>

namespace flow_ros
{

/**
 * @brief An in-process message routing object
 *
 *        Can advertise and subscribe to message topics, and matches
 *        connection-making interfaces of a <code>ros::NodeHandle<code>.
 *        Allows for the ability to connect <code>EventHandlers</code in-process
 *        with data channels that do not pass ROS-serializable messages
 */
class Router
{
  /// Alias for local publication type
  template<typename MsgT>
  using publication_t = routing::LocalPublication<MsgT>;

  /// Alias for local subscription type
  template<typename MsgT>
  using subscription_t = routing::LocalSubscription<std::add_const_t<MsgT>>;

  /// Alias for subscription group resource
  using SubscriptionGroupPtr = std::shared_ptr<routing::LocalSubscriptionGroup>;

public:
  /**
   * @brief Namespace constructor
   *
   * @param ns  namespace to associated with this Router
   */
  explicit Router(std::string ns = "anonymous_local_context");

  /**
   * @brief Returns name associated with this Router
   */
  inline const std::string& getNamespace() const
  {
    return namespace_;
  }

  /**
   * @brief Clears all known subscriptions and publications
   */
  inline void clear()
  {
    local_subscription_mapping_.clear();
    publishers_.clear();
  }

  /**
   * @brief Advertises a new local Publication
   *
   * @tparam MsgT  (deduced) type of message to be published
   *
   * @param topic  topic associated with messages to publish
   * @param queue_size  unused
   * @param latch  unused
   *
   * @return new Publication resource
   */
  template<typename MsgT>
  inline std::shared_ptr<publication_t<MsgT>> advertise(const std::string& topic,
                                                        std::uint32_t queue_size,
                                                        bool latch = false)
  {
    // Lock when adding new connection
    std::lock_guard<std::mutex> lock{connect_mutex_};

    // Create new publisher resource tied to a subscription group
    const auto pub =
      [this, &topic]() -> std::shared_ptr<publication_t<MsgT>>
      {
        // Get resolved topic name
        const std::string resolved_topic = resolveName(topic);

        // Create pub associated with appropriate subscription group
        return std::make_shared<publication_t<MsgT>>(resolved_topic, resolveSubscriptionGroup(resolved_topic));
      }();

    // Add to held publishers
    publishers_.emplace_back(pub);
    return pub;
  }

  /**
   * @brief Creates a new local Subscription
   *
   * @tparam MsgT  (deduced) type of message to be received
   *
   * @param topic  topic associated with messages to publish
   * @param queue_size  unused
   * @param cb  message callback to associated with subscriber
   *
   * @return new Subscription resource
   */
  template<typename MsgT, typename CallbackT>
  inline std::shared_ptr<subscription_t<MsgT>> subscribe(const std::string& topic,
                                                         std::uint32_t queue_size,
                                                         CallbackT&& callback)
  {
    // Lock when adding new connection
    std::lock_guard<std::mutex> lock{connect_mutex_};

    // Get resolved topic name
    const std::string resolved_topic = resolveName(topic);

    // Get subscription group associated with topic
    SubscriptionGroupPtr sub_group = resolveSubscriptionGroup(resolved_topic);

    // Create subscription
    auto sub = std::make_shared<subscription_t<MsgT>>(resolved_topic, std::forward<CallbackT>(callback));

    // Add subscription to group
    sub_group->addSubscription(sub);

    return sub;
  }

  /**
   * @brief Creates a new local Subscription
   *
   *        This is a convenience method for class methods
   *
   * @tparam MsgT  (deduced) type of message to be published
   * @tparam MethodT  (deduced) class method type
   * @tparam ThisT  (deduced) class pointer type
   *
   * @param topic  topic associated with messages to publish
   * @param queue_size  unused
   * @param m_fn_ptr  message callback to associated with subscriber
   * @param this_ptr  object with method <code>m_fn_ptr</code>
   *
   * @return new Subscription resource
   */
  template<typename MsgT, typename MethodT, typename ThisT>
  inline std::shared_ptr<subscription_t<MsgT>> subscribe(const std::string& topic,
                                                         std::uint32_t queue_size,
                                                         MethodT&& m_fn_ptr,
                                                         ThisT&& this_ptr)
  {
    return subscribe<MsgT>(topic, queue_size,
                           std::bind(std::forward<MethodT>(m_fn_ptr),
                                     std::forward<ThisT>(this_ptr),
                                     std::placeholders::_1));
  }

  /**
   * @brief Inject message on a given topic
   *
   * @tparam MsgT  (deduced) type of message to inject
   * @tparam SharedPtrTmpl  (deduced) share pointer object template
   *
   * @param topic  name of local topic to publish on
   * @param msg  message to inject to input (subscription) channel for given topic
   *
   * @throws <code>std::runtime_error</code> if there is no subscription for given topic
   */
  template<typename MsgT, template<typename> class SharedPtrTmpl>
  inline void inject(const std::string& topic, const SharedPtrTmpl<MsgT>& msg)
  {
    // Lock on connection lookup
    std::lock_guard<std::mutex> lock{connect_mutex_};

    // Get fully qualified topic name
    const std::string resolved_topic = resolveName(topic);

    // Get channel handle
    const auto itr = local_subscription_mapping_.find(resolved_topic);

    if (itr == local_subscription_mapping_.end())
    {
      std::ostringstream oss;
      oss << "Unknown subscription for topic: " << topic << " (resolved to=" << resolved_topic << ")"; 
      throw std::runtime_error{oss.str()};
    }
    else
    {
      // Inject message to all local subscription channels for this topic
      itr->second->call<MsgT>(msg);
    }
  }

#ifdef FLOW_ROS_HAS_ROSBAG_SUPPORT

  /**
   * @brief Inject ROS bag message instance on a given topic
   *
   * @param mi  ROS bag message instance
   *
   * @throws <code>std::runtime_error</code> if there is no subscription for given topic
   * @throws <code>std::runtime_error</code> on failure to instance message
   */
  inline void inject(const ::rosbag::MessageInstance& mi)
  {
    // Lock on connection lookup
    std::lock_guard<std::mutex> lock{connect_mutex_};

    // Get channel handle
    const auto itr = local_subscription_mapping_.find(mi.getTopic());

    if (itr == local_subscription_mapping_.end())
    {
      std::ostringstream oss;
      oss << "Unknown subscription for message instance with topic: " << mi.getTopic(); 
      throw std::runtime_error{oss.str()};
    }
    else
    {
      // Inject message to all local subscription channels for this topic
      itr->second->call(mi);
    }
  }

#endif // FLOW_ROS_HAS_ROSBAG_SUPPORT

  /**
   * @brief Returns a vector of known published topics
   */
  std::vector<std::string> knownPublications() const;

  /**
   * @brief Returns a vector of known subscribed topics
   */
  std::vector<std::string> knownSubscriptions() const;

  /**
   * @brief Returns topic name resolved under Router namespace
   * @param topic  topic name to resolve
   * @return resolved version of \p topic
   * @throws <code>std::invalid_argument</code> if \p topic is invalid
   */
  std::string resolveName(const std::string& topic) const;

private:
  /**
   * @brief Returns subscription group from a topic name
   * @param resolved_topic  fully-qualified connection topic name
   * @return subscription group resource
   */
  SubscriptionGroupPtr resolveSubscriptionGroup(const std::string& resolved_topic);

  /// Name associated with this object (typically a node name)
  std::string namespace_;

  /// Collection of currently known publishers
  std::vector<std::shared_ptr<routing::LocalPublicationBase>> publishers_;

  /// Collection of subscriptions mapped to a topic name
  std::unordered_map<std::string, SubscriptionGroupPtr> local_subscription_mapping_;

  /// Mutex which protects Router on new publisher/subscriber connections
  mutable std::mutex connect_mutex_;

  /**
   * @brief Router output stream operator overload
   * @param[in,out] os  output stream
   * @param router  router object
   * @return os
   */
  friend std::ostream& operator<<(std::ostream& os, const Router& router);
};

}  // namespace flow_ros

#endif  // FLOW_ROS_ROUTER_H
