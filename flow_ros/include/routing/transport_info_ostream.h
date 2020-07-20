/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file  transport_info_ostream.h
 */
#ifndef FLOW_ROS_ROUTING_TRANSPORT_INFO_OSTREAM_H
#define FLOW_ROS_ROUTING_TRANSPORT_INFO_OSTREAM_H

// C++ Standard Library
#include <ostream>

namespace flow_ros
{
namespace routing
{

/**
 * @brief Output stream overload for <code>flow_ros::routing::TransportMethod</code>
 *
 * @param[in,out] os  output stream
 * @param transport  transport info enum
 *
 * @return os
 */
inline std::ostream& operator<<(std::ostream& os, const TransportMethod& transport)
{
  switch (transport)
  {
  case TransportMethod::UNKNOWN:
    return os << "UNKNOWN";
  case TransportMethod::MULTI:
    return os << "MULTI";
  case TransportMethod::LOCAL:
    return os << "LOCAL";
  case TransportMethod::ROS:
    return os << "ROS";
  default:
    return os;
  }
}


/**
 * @brief Output stream overload for <code>flow_ros::routing::Direction</code>
 *
 * @param[in,out] os  output stream
 * @param direction  direction info enum
 *
 * @return os
 */
inline std::ostream& operator<<(std::ostream& os, const Direction& direction)
{
  switch (direction)
  {
  case Direction::UNKNOWN:
    return os << "UNKNOWN";
  case Direction::INBOUND:
    return os << "INBOUND";
  case Direction::OUTBOUND:
    return os << "OUTBOUND";
  default:
    return os;
  }
}

}  // namespace routing
}  // namespace flow_ros

#endif  // FLOW_ROS_ROUTING_TRANSPORT_INFO_OSTREAM_H
