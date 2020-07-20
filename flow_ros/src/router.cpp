/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @file phonebook.cpp
 * @brief Implmentation for singleton default-Router object
 */

// C++ Standard Library
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

// Flow
#include <flow_ros/router.h>

namespace flow_ros
{

Router::Router(std::string ns) : namespace_{std::move(ns)}
{
  while (!namespace_.empty() and namespace_.back() == '/')
  {
    namespace_.pop_back();
  }
}


std::vector<std::string> Router::knownPublications() const
{
  // Lock on connection lookup
  std::lock_guard<std::mutex> lock{connect_mutex_};

  std::vector<std::string> topics;
  topics.reserve(publishers_.size());

  for (const auto& p : publishers_)
  {
    topics.emplace_back(p->getTopic());
  }
  return topics;
}


std::vector<std::string> Router::knownSubscriptions() const
{
  // Lock on connection lookup
  std::lock_guard<std::mutex> lock{connect_mutex_};

  std::vector<std::string> topics;
  topics.reserve(local_subscription_mapping_.size());

  for (const auto& ls : local_subscription_mapping_)
  {
    topics.emplace_back(ls.first);
  }
  return topics;
}


std::string Router::resolveName(const std::string& topic) const
{
  if (topic.empty())
  {
    throw std::invalid_argument{"Empty topic name is invalid"};
  }

  const auto ns = getNamespace();
  if (topic.find(ns) == 0)
  {
    return topic;
  }
  else if (topic.front() == '/')
  {
    return topic;
  }
  else if (topic.back() == '/')
  {
    return ns + topic;
  }
  return ns + "/" + topic;
}


Router::SubscriptionGroupPtr Router::resolveSubscriptionGroup(const std::string& resolved_topic)
{
  // Check if any corresponding subscription groups are already available
  auto lsg_itr = local_subscription_mapping_.find(resolved_topic);
  if (lsg_itr != local_subscription_mapping_.end())
  {
    return lsg_itr->second;
  }

  // Create new sub group
  SubscriptionGroupPtr group = std::make_shared<routing::LocalSubscriptionGroup>();

  // Add it to sub group map
  return local_subscription_mapping_.emplace(resolved_topic, std::move(group)).first->second;
}


std::ostream& operator<<(std::ostream& os, const Router& router)
{
  os << router.getNamespace() << "\n";

  os << std::setw(20) << "  Known Publications:"
     << "\n";
  for (const auto pub : router.publishers_)
  {
    os << std::setw(10) << ":> " << pub->getTopic() << "\n";
  }

  os << std::setw(20) << "  Known Subscriptions:"
     << "\n";
  for (const auto sub : router.local_subscription_mapping_)
  {
    os << std::setw(10) << ":> " << sub.first << " (";
    os << sub.second->size() << " subscribed)"
       << "\n";
  }

  return os;
}

}  // namespace flow_ros
