/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file  publication.h
 */
#ifndef FLOW_ROS_ROUTING_LOCAL_PUBLICATION_H
#define FLOW_ROS_ROUTING_LOCAL_PUBLICATION_H

// C++ Standard Library
#include <memory>
#include <string>
#include <utility>

// Flow
#include <flow_ros/message_ptr.h>
#include <flow_ros/routing/local_subscription.h>
#include <flow_ros/routing/publication_wrapper.h>
#include <flow_ros/routing/transport_info.h>


namespace flow_ros
{
namespace routing
{

/**
 * @brief LocalPublication base which provides basic info and erases MsgT-dependent methods
 */
class LocalPublicationBase
{
public:
  virtual ~LocalPublicationBase() = default;

  /**
   * @brief Topic setup constructor
   *
   * @param topic  topic name to associate with this object
   */
  explicit LocalPublicationBase(std::string topic) : topic_{std::move(topic)} {}

  /**
   * @brief Returns topic name associated with this object
   */
  inline const std::string& getTopic() const { return topic_; }

private:
  /// Topic name
  std::string topic_;
};


/**
 * @brief A Subscription object used for passing messages locally
 *
 * @tparam MsgT  transported data type
 */
template <typename MsgT> class LocalPublication final : public LocalPublicationBase, public PublicationWrapper<MsgT>
{
public:
  /**
   * @brief Required setup constructor
   *
   * @param topic  topic associated with published messages
   * @param subscribers  group of LocalSubscription object which will be passed published messages
   */
  LocalPublication(std::string topic, std::shared_ptr<LocalSubscriptionGroup> subscribers) :
      LocalPublicationBase{std::move(topic)},
      subscribers_{std::move(subscribers)}
  {}

  virtual ~LocalPublication() = default;

  /**
   * @brief Call all held subscriber callbacks on message being published
   *
   * @param message  message data to publish
   */
  void publish(const message_shared_ptr_t<MsgT>& message) const { subscribers_->call<MsgT>(message); }

  /**
   * @brief Returns topic associated with publication
   */
  std::string getTopic() const override { return LocalPublicationBase::getTopic(); }

  /**
   * @brief Returns number of local subscriptions connected to this LocalPublication
   */
  std::uint32_t getNumSubscribers() const override { return subscribers_ ? subscribers_->size() : 0; }

  /**
   * @brief Returns transport method (code) associated with this publisher
   */
  TransportMethod getTransportMethod() const override { return TransportMethod::LOCAL; }

  /**
   * @brief Validation cast operator
   */
  bool isValid() const override { return static_cast<bool>(subscribers_); }

private:
  /// Local subscription objects to pass published data to
  std::shared_ptr<LocalSubscriptionGroup> subscribers_;
};

}  // namespace routing
}  // namespace flow_ros


#endif  // FLOW_ROS_ROUTING_LOCAL_PUBLICATION_H
