/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file  publication.h
 */
#ifndef FLOW_ROS_ROUTING_PUBLICATION_WRAPPER_H
#define FLOW_ROS_ROUTING_PUBLICATION_WRAPPER_H

// C++ Standard Library
#include <cstdint>
#include <string>

// Flow
#include <flow_ros/message_ptr.h>
#include <flow_ros/routing/transport_info.h>

namespace flow_ros
{
namespace routing
{

/**
 * @brief CRTP-base for message publishing objects
 */
template <typename MsgT> class PublicationWrapper
{
public:
  virtual ~PublicationWrapper() = default;

  /**
   * @brief Publishes message
   *
   * @param message  message data to publish
   */
  virtual void publish(const message_shared_ptr_t<MsgT>& message) const = 0;

  /**
   * @brief Returns topic associated with publication
   */
  virtual std::string getTopic() const = 0;

  /**
   * @brief Returns number of local subscriptions connected to this publication
   */
  virtual std::uint32_t getNumSubscribers() const = 0;

  /**
   * @brief Returns flag indicating whether or not publisher is latched
   */
  virtual bool isLatched() const = 0;

  /**
   * @brief Returns transport method (code) associated with this publication
   */
  virtual TransportMethod getTransportMethod() const = 0;

  /**
   * @brief Returns transport direction (code) associated with this publication
   */
  static constexpr Direction getTransportDirection() { return Direction::OUTBOUND; }

  /**
   * @brief Validity check
   */
  virtual bool isValid() const = 0;

  /**
   * @brief Validity check cast
   */
  virtual operator bool() const { return isValid(); };
};

}  // namespace routing
}  // namespace flow_ros

// Flow (forward declaration fulfillment)
#include <flow_ros/routing/ros_publication.h>

#endif  // FLOW_ROS_ROUTING_PUBLICATION_WRAPPER_H
