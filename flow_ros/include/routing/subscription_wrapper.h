/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file  subscription_wrapper.h
 */
#ifndef FLOW_ROS_ROUTING_SUBSCRIPTION_WRAPPER_H
#define FLOW_ROS_ROUTING_SUBSCRIPTION_WRAPPER_H

// C++ Standard Library
#include <cstdint>
#include <string>

// Flow
#include <flow/impl/implement_crtp_base.hpp>
#include <flow_ros/routing/transport_info.h>

namespace flow_ros
{
namespace routing
{

/**
 * @brief CRTP-base for message subscribing objects
 */
class SubscriptionWrapper
{
public:
  virtual ~SubscriptionWrapper() = default;

  /**
   * @brief Returns topic associated with publication
   */
  virtual std::string getTopic() const = 0;

  /**
   * @brief Returns number of local publications connected to this subscription
   */
  virtual std::uint32_t getNumPublishers() const = 0;

  /**
   * @brief Returns transport method (code) associated with this subscription
   */
  virtual TransportMethod getTransportMethod() const = 0;

  /**
   * @brief Returns transport direction (code) associated with this subscription
   */
  static constexpr Direction getTransportDirection()
  {
    return Direction::INBOUND;
  }

  /**
   * @brief Validity check
   */
  virtual bool isValid() const = 0;

  /**
   * @brief Validity check cast
   */
  virtual operator bool() const
  {
    return isValid();
  };
};

}  // namespace routing
}  // namespace flow_ros

#endif  // FLOW_ROS_ROUTING_SUBSCRIPTION_WRAPPER_H
