/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 */

// C++ Standard Library
#include <cstdint>

// GTest
#include <gtest/gtest.h>

// ROS Messages
#include <std_msgs/Header.h>

// Flow
#include <flow/flow.h>
#include <flow_ros/subscriber.h>
#include <flow_ros/router.h>


/// A ROS messages-like object
struct TestMessage
{
  using Ptr = std::shared_ptr<TestMessage>;
  using ConstPtr = std::shared_ptr<const TestMessage>;

  std_msgs::Header header;
};


TEST(SubscriberLocal, DrivingPolicyDefault)
{
  flow_ros::Router router{"/router"};

  flow_ros::Subscriber<TestMessage, flow::driver::Next, flow::NoLock> sub{router, "topic", 1};

  EXPECT_EQ(sub.getNumPublishers(), 0UL);
  EXPECT_EQ(sub.getTransportMethod(), flow_ros::routing::TransportMethod::LOCAL);
  EXPECT_EQ(sub.getTopic(), "/router/topic");
  EXPECT_EQ(sub.size(), 0UL);
  EXPECT_EQ(sub.getCapacity(), 0UL);
}


TEST(SubscriberLocal, FollowingPolicyDefault)
{
  flow_ros::Router router{"/router"};

  flow_ros::Subscriber<TestMessage, flow::follower::Before, flow::NoLock> sub{router, "topic", 1, ros::Duration{0}};

  EXPECT_EQ(sub.getNumPublishers(), 0UL);
  EXPECT_EQ(sub.getTransportMethod(), flow_ros::routing::TransportMethod::LOCAL);
  EXPECT_EQ(sub.getTopic(), "/router/topic");
  EXPECT_EQ(sub.size(), 0UL);
  EXPECT_EQ(sub.getCapacity(), 0UL);
}
