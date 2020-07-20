/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file  transport_info.h
 */
#ifndef FLOW_ROS_ROUTING_TRANSPORT_INFO_H
#define FLOW_ROS_ROUTING_TRANSPORT_INFO_H

namespace flow_ros
{
namespace routing
{

/**
 * @brief Transport method codes
 */
enum class TransportMethod : int
{
  UNKNOWN,  ///< Unknown transport method (default)
  MULTI,  ///< Messages are published over multiple connections
  LOCAL,  ///< Message is published over a local connection (Router)
  ROS,  ///< ROS messaging publication
};


/**
 * @brief Transport direction codes
 */
enum class Direction : int
{
  UNKNOWN,  ///< Unknown transport direction
  INBOUND,  ///< Inbound transport direction
  OUTBOUND,  ///< Outbound transport direction
};

}  // namespace routing
}  // namespace flow_ros

#endif  // FLOW_ROS_ROUTING_TRANSPORT_INFO_H
