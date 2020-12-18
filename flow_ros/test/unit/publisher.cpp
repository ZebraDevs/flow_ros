/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 */

// C++ Standard Library
#include <cstdint>
#include <vector>

// GTest
#include <gtest/gtest.h>

// ROS
#include <ros/time.h>

// ROS Messages
#include <std_msgs/Header.h>

// Flow
#include <flow/flow.h>
#include <flow_ros/message_stamp_access.h>
#include <flow_ros/publisher.h>
#include <flow_ros/router.h>


/// A ROS messages-like object
struct TestMessage
{
  using Ptr = std::shared_ptr<TestMessage>;
  using ConstPtr = std::shared_ptr<const TestMessage>;

  std_msgs::Header header;
};


TEST(PublisherLocal, Default)
{
  flow_ros::Router router{"/router"};

  flow_ros::Publisher<TestMessage> pub{router, "topic", 1};

  EXPECT_EQ(pub.getNumSubscribers(), 0UL);
  EXPECT_EQ(pub.getTransportMethod(), flow_ros::routing::TransportMethod::LOCAL);
  EXPECT_EQ(pub.getTopic(), "/router/topic");
}


TEST(PublisherLocal, PublishSingleInvalid)
{
  flow_ros::Router router{"/router"};

  flow_ros::Publisher<TestMessage> pub{router, "topic", 1};

  std::size_t counter = 0;
  auto sub = router.subscribe<TestMessage>("topic", 1, [&counter](const TestMessage::ConstPtr& msg) { ++counter; });

  pub.publish(TestMessage::ConstPtr{});

  ASSERT_EQ(counter, 0UL);
}


TEST(PublisherLocal, PublishSingleValid)
{
  flow_ros::Router router{"/router"};

  flow_ros::Publisher<TestMessage> pub{router, "topic", 1};

  std::size_t counter = 0;
  auto sub = router.subscribe<TestMessage>("topic", 1, [&counter](const TestMessage::ConstPtr& msg) { ++counter; });

  pub.publish(std::make_shared<TestMessage>());

  ASSERT_EQ(counter, 1UL);
}


TEST(PublisherLocal, PublishMultiValid)
{
  flow_ros::Router router{"/router"};

  flow_ros::MultiPublisher<const TestMessage> pub{router, "topic", 1};

  std::vector<TestMessage::ConstPtr> sub_msgs;
  auto sub = router.subscribe<TestMessage>(
    "topic", 1, [&sub_msgs](const TestMessage::ConstPtr& msg) { sub_msgs.emplace_back(msg); });

  const std::vector<TestMessage::Ptr> pub_msgs{
    std::make_shared<TestMessage>(),
    std::make_shared<TestMessage>(),
    std::make_shared<TestMessage>(),
  };

  pub.publish(pub_msgs.begin(), pub_msgs.end());

  ASSERT_EQ(sub_msgs.size(), pub_msgs.size());
}
