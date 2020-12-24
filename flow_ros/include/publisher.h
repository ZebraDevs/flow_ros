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
#include <type_traits>
#include <vector>

// ROS
#include <ros/node_handle.h>
#include <ros/publisher.h>

// Flow
#include <flow/impl/static_assert.hpp>
#include <flow_ros/message_ptr.h>
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
   * @copydoc routing::PublicationWrapper::getTopic
   */
  virtual std::string getTopic() const = 0;

  /**
   * @copydoc routing::PublicationWrapper::getNumSubscribers
   */
  virtual std::uint32_t getNumSubscribers() const = 0;

  /**
   * @copydoc routing::PublicationWrapper::isLatched
   */
  virtual bool isLatched() const = 0;

  /**
   * @copydoc routing::PublicationWrapper::getTransportMethod
   */
  virtual routing::TransportMethod getTransportMethod() const = 0;

  /**
   * @copydoc routing::PublicationWrapper::isValid
   */
  virtual bool isValid() const = 0;
};


/**
 * @brief Message publisher base type
 *
 * @tparam MsgT  Message type
 */
template <typename MsgT> class PublisherOutputBase : public PublisherBase
{
public:
  /**
   * @copydoc routing::PublicationWrapper::getTopic
   */
  std::string getTopic() const final { return publication_->getTopic(); }

  /**
   * @copydoc routing::PublicationWrapper::getNumSubscribers
   */
  std::uint32_t getNumSubscribers() const final { return publication_->getNumSubscribers(); }

  /**
   * @copydoc routing::PublicationWrapper::isLatched
   */
  bool isLatched() const final { return publication_->isLatched(); }

  /**
   * @copydoc routing::PublicationWrapper::getTransportMethod
   */
  routing::TransportMethod getTransportMethod() const final { return publication_->getTransportMethod(); }

  /**
   * @copydoc routing::PublicationWrapper::isValid
   */
  bool isValid() const final { return publication_->isValid(); }

  /**
   * @copydoc routing::PublicationWrapper::getTransportDirection
   */
  static constexpr routing::Direction getTransportDirection()
  {
    return routing::PublicationWrapper<MsgT>::getTransportDirection();
  }

protected:
  /**
   * @brief Publication setup constructor (generic)
   * @param pub  publication resource
   */
  explicit PublisherOutputBase(std::shared_ptr<routing::PublicationWrapper<MsgT>> pub) : publication_{std::move(pub)} {}

  /**
   * @brief Call all held subscriber callbacks on message being published
   *
   * @param message  message data to publish
   */
  inline void publish(const message_shared_const_ptr_t<MsgT>& message) const { publication_->publish(message); }

private:
  /// Message publisher
  std::shared_ptr<routing::PublicationWrapper<MsgT>> publication_;
};


/**
 * @brief Output channel which publishes messages
 *
 * @tparam MsgT  message type
 */
template <typename MsgT> class Publisher final : public PublisherOutputBase<MsgT>
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
  Publisher(ros::NodeHandle& nh, const std::string& topic, std::uint32_t queue_size = 0, const bool latched = false) :
      PublisherOutputBaseType{
        std::make_shared<routing::ROSPublication<MsgT>>(nh.advertise<MsgT>(topic, queue_size, latched))}
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
  Publisher(Router& pb, std::string topic, std::uint32_t queue_size = 0, const bool latched = false) :
      PublisherOutputBaseType{pb.advertise<MsgT>(std::move(topic), queue_size)}
  {}

  /**
   * @brief Publishes output message
   *
   *        If message resource is invalid (i.e. <code>msg == nullptr</code>), then
   *        no message is sent over underlying publication channel
   *
   * @param message  next output message
   */
  inline void publish(const message_shared_const_ptr_t<MsgT>& message) const
  {
    if (static_cast<bool>(message))
    {
      PublisherOutputBaseType::publish(message);
    }
  }
};


/**
 * @brief Output channel specialization which publishes multiple messages
 *
 * @tparam MsgT  message type
 * @tparam OutputContainerT  container of message resources used as <code>OutputType</code>;
 *                           this information is most relevant when using with MultiPublisher with EventHandler
 */
template <typename MsgT, typename OutputContainerT = std::vector<message_shared_ptr_t<MsgT>>>
class MultiPublisher final : public PublisherOutputBase<MsgT>
{
  /// Subscriber base type alias
  using PublisherOutputBaseType = PublisherOutputBase<MsgT>;

public:
  FLOW_STATIC_ASSERT(std::is_const<MsgT>(), "MsgT must be const");

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
  MultiPublisher(
    ros::NodeHandle& nh,
    const std::string& topic,
    const std::uint32_t queue_size = 0,
    const bool latched = false) :
      PublisherOutputBaseType{
        std::make_shared<routing::ROSPublication<MsgT>>(nh.advertise<MsgT>(topic, queue_size, latched))}
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
  MultiPublisher(Router& pb, std::string topic, const std::uint32_t queue_size = 0) :
      PublisherOutputBaseType{pb.advertise<MsgT>(std::move(topic), queue_size)}
  {}

  /**
   * @brief Publishes range of messages
   * @param first  iterator to first message to publish
   * @param last  one past last message iterator
   */
  template <typename MsgPtrIteratorT> inline void publish(MsgPtrIteratorT first, const MsgPtrIteratorT last) const
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
   * @param messages  container of messages
   */
  inline void publish(const OutputContainerT& messages) const { publish(std::begin(messages), std::end(messages)); }
};


/**
 * @brief Helper function use to set message sequencing stamp
 *
 * @tparam MsgT  (deduced) message type
 * @tparam SeqT  (deduced) sequencing type
 *
 * @param msg  message
 * @param seq  message sequence counter
 */
template <typename MsgT, typename SeqT> inline void set_stamp(MsgT&& msg, SeqT&& seq)
{
  using CleanedMsgT = std::remove_reference_t<MsgT>;
  StampSetter<CleanedMsgT>{}(std::forward<MsgT>(msg), std::forward<SeqT>(seq));
}


/**
 * @brief Publisher type traits
 */
template <typename MsgT> struct PublisherTraits;


/**
 * @copydoc PublisherTraits
 * @note Publisher partial specialization
 */
template <typename MsgT> struct PublisherTraits<Publisher<MsgT>>
{
  /// Output message type
  using MsgType = MsgT;

  /// Output message resource type
  using OutputType = message_shared_ptr_t<MsgT>;
};


/**
 * @copydoc PublisherTraits
 * @note MultiPublisher partial specialization
 */
template <typename MsgT, typename OutputContainerT> struct PublisherTraits<MultiPublisher<MsgT, OutputContainerT>>
{
  /// Output message type
  using MsgType = MsgT;

  /// Output message resource type
  using OutputType = OutputContainerT;
};

}  // namespace flow_ros

#endif  // FLOW_ROS_PUBLISHER_H
