/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file event_handler.h
 */
#ifndef FLOW_ROS_EVENT_HANDLER_OSTREAM_H
#define FLOW_ROS_EVENT_HANDLER_OSTREAM_H

// C++ Standard Library
#include <iostream>

// Flow
#include <flow_ros/event_handler.h>

namespace flow_ros
{

/**
 * @brief <code>std::ostream</code> overload for EventSummary::State
 */
inline std::ostream& operator<<(std::ostream& os, const EventSummary::State state)
{
  switch(state)
  {
    case EventSummary::State::UNKNOWN            : return os << "UNKNOWN";
    case EventSummary::State::EXECUTED           : return os << "EXECUTED";
    case EventSummary::State::EXECUTION_BYPASSED : return os << "EXECUTION_BYPASSED";
    case EventSummary::State::SYNC_NEEDS_RETRY   : return os << "SYNC_NEEDS_RETRY";
    case EventSummary::State::SYNC_ABORTED       : return os << "SYNC_ABORTED";
    case EventSummary::State::SYNC_TIMED_OUT     : return os << "SYNC_TIMED_OUT";
    default: break;
  };
  return os << "<Invalid EventSummary::State>";
}


/**
 * @brief <code>std::ostream</code> overload for EventSummary
 */
inline std::ostream& operator<<(std::ostream& os, const EventSummary& summary)
{
  return os << "state: " << summary.state << ", range: (" << summary.range << ')';
}

}  // namespace flow_ros

#endif  // FLOW_ROS_EVENT_HANDLER_OSTREAM_H
