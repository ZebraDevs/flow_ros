/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file publisher.h
 */
#ifndef FLOW_ROS_PUBLISHER_H
#define FLOW_ROS_PUBLISHER_H

// C++ Standard Library
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// ROS
#include <ros/node_handle.h>
#include <ros/publisher.h>

// Flow
#include <flow_ros/message.h>
#include <flow_ros/router.h>
#include <flow_ros/routing/local_publication.h>
#include <flow_ros/routing/publication_wrapper.h>
#include <flow_ros/routing/ros_publication.h>

namespace flow_ros
{

/**
 * @brief Publish base to expose meta-information methods
 */
class PublisherBase
{
public:
  virtual ~PublisherBase() = default;

  /**
   * @brief Returns topic associated with publication
   */
  virtual std::string getTopic() const = 0;

  /**
   * @brief Returns number of local subscriptions connected to this publisher
   */
  virtual std::uint32_t getNumSubscribers() const = 0;

  /**
   * @brief Returns transport method (code) associated with this publisher
   */
  virtual routing::TransportMethod getTransportMethod() const = 0;

  /**
   * @brief Validation cast operator
   */
  virtual bool isValid() const = 0;
};


/**
 * @brief Message publisher base type
 *
 * @tparam MsgT  Message type
 */
template<typename MsgT>
class PublisherOutputBase : public PublisherBase
{
public:
  /**
   * @brief Returns topic associated with publication
   */
  std::string getTopic() const final
  {
    return publication_->getTopic();
  }

  /**
   * @brief Returns number of local subscriptions connected to this publisher
   */
  std::uint32_t getNumSubscribers() const final
  {
    return publication_->getNumSubscribers();
  }

  /**
   * @brief Returns transport method (code) associated with this publisher
   */
  routing::TransportMethod getTransportMethod() const final
  {
    return publication_->getTransportMethod();
  }

  /**
   * @brief Returns transport direction (code) associated with this publisher
   */
  static constexpr routing::Direction getTransportDirection()
  {
    return routing::PublicationWrapper<MsgT>::getTransportDirection();
  }

  /**
   * @brief Validation cast operator
   */
  bool isValid() const final
  {
    return publication_->isValid();
  }

protected:
  /**
   * @brief Publication setup constructor (generic)
   * @param pub  publication resource
   */
  explicit PublisherOutputBase(std::shared_ptr<routing::PublicationWrapper<MsgT>> pub) :
    publication_{std::move(pub)}
  {}

  /**
   * @brief Call all held subscriber callbacks on message being published
   *
   * @param message  message data to publish
   */
  inline void publish(const message_shared_ptr_t<MsgT>& message) const
  {
    publication_->publish(message);
  }

private:
  /// Message publisher
  std::shared_ptr<routing::PublicationWrapper<MsgT>> publication_;
};


/**
 * @brief Output channel which publishes messages
 *
 * @tparam MsgT  message type
 */
template<typename MsgT>
class Publisher : public PublisherOutputBase<MsgT>
{
  /// Subscriber base type alias
  using PublisherOutputBaseType = PublisherOutputBase<MsgT>;
public:
  /**
   * @brief <code>ros::NodeHandle</code> setup constructor (extra-node messaging)
   *
   * @param[in,out] nh  ROS node handle to negotiate topic advertisement/connection
   * @param topic  topic associated with messages to be output
   * @param queue_size  underlying ROS publisher queue size
   * @param latched  options for creating a "latched" output publisher
   *
   * @warning Will throw with <code>std::runtime_error</code> if underlying publisher
   *          could not be establish a connection with ROS-master
   */
  Publisher(ros::NodeHandle& nh, std::string topic, std::uint32_t queue_size = 0, bool latched = false) :
    PublisherOutputBaseType
    {
      std::make_shared<routing::ROSPublication<MsgT>>(
        nh.advertise<MsgT>(std::move(topic), queue_size, latched))
    }
  {}

  /**
   * @brief <code>Router</code> setup constructor (intra-node messaging)
   *
   * @param[in,out] pb  Intra-node Router routing object
   * @param topic  topic associated with messages to be output
   * @param queue_size  underlying publication queue size
   * @param latched  not used; dummy for API consistency
   *
   * @warning Will throw with <code>std::runtime_error</code> if underlying publisher
   *          could not be establish a connection with ROS-master
   */
  Publisher(Router& pb, std::string topic, std::uint32_t queue_size = 0, bool latched = false) :
    PublisherOutputBaseType
    {
      pb.advertise<MsgT>(std::move(topic), queue_size)
    }
  {}

  /**
   * @brief Publishes output message
   *
   *        If message resource is invalid (i.e. <code>msg == nullptr</code>), then
   *        no message is sent over underlying publication channel
   *
   * @param msg  next output message
   */
  inline void publish(message_shared_ptr_t<MsgT> msg) const
  {
    if (static_cast<bool>(msg))
    {
      PublisherOutputBaseType::publish(std::move(msg));
    }
  }
};


/**
 * @brief Output channel specialization which publishes multiple messages
 *
 * @tparam MsgT  message type
 */
template<typename MsgT>
class MultiPublisher : public PublisherOutputBase<MsgT>
{
  /// Subscriber base type alias
  using PublisherOutputBaseType = PublisherOutputBase<MsgT>;
public:
  /**
   * @brief <code>ros::NodeHandle</code> setup constructor (extra-node messaging)
   *
   * @param[in,out] nh  ROS node handle to negotiate topic advertisement/connection
   * @param topic  topic associated with messages to be output
   * @param queue_size  underlying ROS publisher queue size
   * @param latched  options for creating a "latched" output publisher
   *
   * @warning Will throw with <code>std::runtime_error</code> if underlying publisher
   *          could not be establish a connection with ROS-master
   */
  MultiPublisher(ros::NodeHandle& nh,
                 std::string topic,
                 const std::uint32_t queue_size = 0,
                 const bool latched = false) :
    PublisherOutputBaseType
    {
      std::make_shared<routing::ROSPublication<MsgT>>(
        nh.advertise<MsgT>(std::move(topic), queue_size, latched))
    }
  {}

  /**
   * @brief <code>Router</code> setup constructor (intra-node messaging)
   *
   * @param[in,out] pb  Intra-node Router routing object
   * @param topic  topic associated with messages to be output
   * @param queue_size  underlying publication queue size
   *
   * @warning Will throw with <code>std::runtime_error</code> if underlying publisher
   *          could not be establish a connection with ROS-master
   */
  MultiPublisher(Router& pb,
                 std::string topic,
                 const std::uint32_t queue_size = 0) :
    PublisherOutputBaseType
    {
      pb.advertise<MsgT>(std::move(topic), queue_size)
    }
  {}

  /**
   * @brief Publishes range of messages
   * @param first  iterator to first message to publish
   * @param last  one past last message iterator
   */
  template<typename MsgPtrIteratorT>
  inline void publish(MsgPtrIteratorT first, const MsgPtrIteratorT last) const
  {
    for (/*empty*/; first != last; ++first)
    {
      PublisherOutputBaseType::publish(*first);
    }
  }

  /**
   * @brief Publishes range of messages
   *
   *        If any message resource is invalid (i.e. <code>msg == nullptr</code>), then
   *        no message is sent over underlying publication channel
   *
   * @param messages  vector of messages
   */
  inline void publish(std::vector<message_shared_ptr_t<MsgT>> messages) const
  {
    publish(messages.begin(), messages.end());
  }
};


// Forward declaration
template<typename MsgT>
struct PublisherTraits;


/**
 * @brief Publisher type traits
 */
template<typename MsgT>
struct PublisherTraits<Publisher<MsgT>>
{
  /// Output message type
  using MsgType = MsgT;

  /// Output message type
  using OutputType = message_shared_ptr_t<MsgT>;
};


/**
 * @brief Publisher type traits
 */
template<typename MsgT>
struct PublisherTraits<MultiPublisher<MsgT>>
{
  /// Output message type
  using MsgType = MsgT;

  /// Output message type
  using OutputType = std::vector<message_shared_ptr_t<MsgT>>;
};

}  // namespace flow_ros

#endif  // FLOW_ROS_PUBLISHER_H
