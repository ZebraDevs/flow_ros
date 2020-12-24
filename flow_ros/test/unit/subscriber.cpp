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
#include <flow_ros/message_stamp_access.h>
#include <flow_ros/router.h>
#include <flow_ros/subscriber.h>


// A ROS message-like object
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


TEST(SubscriberLocal, MessageInjectSameStamp)
{
  flow_ros::Router router{"/router"};

  flow_ros::Subscriber<TestMessage, flow::follower::Before, flow::NoLock> sub{router, "topic", 1, ros::Duration{0}};

  ASSERT_EQ(sub.get_capacity(), 0UL);

  for (int i = 0; i < 10; ++i)
  {
    sub.inject([] {
      auto msg = std::make_shared<TestMessage>();
      msg->header.stamp.fromSec(1);
      return msg;
    }());
  }

  EXPECT_EQ(sub.size(), 1UL);
}


TEST(SubscriberLocal, MessageInjectSequence)
{
  flow_ros::Router router{"/router"};

  flow_ros::Subscriber<TestMessage, flow::follower::Before, flow::NoLock> sub{router, "topic", 1, ros::Duration{0}};

  ASSERT_EQ(sub.get_capacity(), 0UL);

  for (int i = 0; i < 2; ++i)
  {
    sub.inject([i] {
      auto msg = std::make_shared<TestMessage>();
      msg->header.stamp.fromSec(i);
      return msg;
    }());
  }

  EXPECT_EQ(sub.size(), 2UL);
}


TEST(SubscriberLocal, MessageInjectFixedBufferCapacity)
{
  flow_ros::Router router{"/router"};

  flow_ros::Subscriber<TestMessage, flow::follower::Before, flow::NoLock> sub{router, "topic", 1, ros::Duration{0}};

  sub.set_capacity(3UL);
  ASSERT_EQ(sub.get_capacity(), 3UL);

  for (int i = 0; i < 10; ++i)
  {
    sub.inject([i] {
      auto msg = std::make_shared<TestMessage>();
      msg->header.stamp.fromSec(i);
      return msg;
    }());
  }

  EXPECT_EQ(sub.size(), 3UL);
}


TEST(SubscriberLocal, MessageRemove)
{
  flow_ros::Router router{"/router"};

  flow_ros::Subscriber<TestMessage, flow::follower::Before, flow::NoLock> sub{router, "topic", 1, ros::Duration{0}};

  ASSERT_EQ(sub.get_capacity(), 0UL);

  for (int i = 0; i < 10; ++i)
  {
    sub.inject([i] {
      auto msg = std::make_shared<TestMessage>();
      msg->header.stamp.fromSec(i);
      return msg;
    }());
  }

  sub.remove(ros::Time{2});

  // Removed all before 2, leaving 8 more
  EXPECT_EQ(sub.size(), 8UL);
}


TEST(SubscriberLocal, MessageCapture)
{
  flow_ros::Router router{"/router"};

  flow_ros::Subscriber<TestMessage, flow::follower::Before, flow::NoLock> sub{router, "topic", 1, ros::Duration{0}};

  ASSERT_EQ(sub.get_capacity(), 0UL);

  sub.inject([] {
    auto msg = std::make_shared<TestMessage>();
    msg->header.stamp.fromSec(1);
    return msg;
  }());

  sub.inject([] {
    auto msg = std::make_shared<TestMessage>();
    msg->header.stamp.fromSec(2);
    return msg;
  }());

  std::vector<TestMessage::ConstPtr> messages;
  sub.capture(std::back_inserter(messages), flow::CaptureRange<ros::Time>{ros::Time{2}, ros::Time{2}});

  EXPECT_EQ(sub.size(), 1UL);
  EXPECT_EQ(messages.size(), 1UL);
}


// A ROS message-like object
struct TestMessageWithStamp
{
  using Ptr = std::shared_ptr<TestMessageWithStamp>;
  using ConstPtr = std::shared_ptr<const TestMessageWithStamp>;

  ros::Time stamp;
};


TEST(SubscriberLocal, MessageCaptureDefaultAccessStamp)
{
  flow_ros::Router router{"/router"};

  flow_ros::Subscriber<TestMessageWithStamp, flow::follower::Before, flow::NoLock> sub{
    router, "topic", 1, ros::Duration{0}};

  ASSERT_EQ(sub.get_capacity(), 0UL);

  sub.inject([] {
    auto msg = std::make_shared<TestMessageWithStamp>();
    msg->stamp.fromSec(1);
    return msg;
  }());

  sub.inject([] {
    auto msg = std::make_shared<TestMessageWithStamp>();
    msg->stamp.fromSec(2);
    return msg;
  }());

  std::vector<TestMessageWithStamp::ConstPtr> messages;
  sub.capture(std::back_inserter(messages), flow::CaptureRange<ros::Time>{ros::Time{2}, ros::Time{2}});

  EXPECT_EQ(sub.size(), 1UL);
  EXPECT_EQ(messages.size(), 1UL);
}
