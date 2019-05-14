/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 */

// C++ Standard Library
#include <atomic>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

// GTest
#include <gtest/gtest.h>

// Flow
#include <flow_ros/router.h>


/// A ROS messages-like object
struct TestMessage
{
  using Ptr = std::shared_ptr<TestMessage>;
  using ConstPtr = std::shared_ptr<const TestMessage>;
};


/// A callback function, not associated with an object
static void test_callback(const TestMessage::ConstPtr& msg)
{
  // does nothing
}


/// An object with a callback function
struct CallbackObject
{
  CallbackObject(flow_ros::Router& pb, const std::string topic) :
    callback_count_{0},
    object_is_valid_{true}
  {
    subscription_ = pb.subscribe<TestMessage>(topic, 1, &CallbackObject::callback, this);
  }

  ~CallbackObject()
  {
    // releasing the subscription should prevent callback from being called
    subscription_.reset();

    // At this point the object is no-longer valid and should not be called
    object_is_valid_ = false;
  }

  void callback(const TestMessage::ConstPtr& msg)
  {
    if (!object_is_valid_)
    {
      throw std::runtime_error{"CallbackObject::callback called on an invalid object"};
    }
    ++callback_count_;
  }

  unsigned callback_count_ = 0;
  std::atomic<bool> object_is_valid_;
  std::shared_ptr<flow_ros::routing::LocalSubscriptionBase> subscription_;
};


TEST(Router, ResolveName)
{
  flow_ros::Router router{"/router"};

  ASSERT_EQ(router.resolveName("topic"), "/router/topic");
}


TEST(Router, ResolveNameAbsolute)
{
  flow_ros::Router router{"/router"};

  ASSERT_EQ(router.resolveName("/topic"), "/topic");
}


TEST(Router, ResolveNameEmpty)
{
  flow_ros::Router router{"/router"};

  ASSERT_THROW(router.resolveName(""), std::invalid_argument);
}


TEST(Router, AdvertiseThenSubscribe)
{
  flow_ros::Router router{"/router"};

  auto new_p = router.advertise<TestMessage>("https://goo.gl/DkzYWB", 1);
  ASSERT_TRUE(static_cast<bool>(new_p));

  auto new_s = router.subscribe<TestMessage>("https://goo.gl/DkzYWB", 1, test_callback);
  ASSERT_TRUE(static_cast<bool>(new_s));
}


TEST(Router, SubscribeThenAdvertise)
{
  flow_ros::Router router{"/router"};

  auto new_s = router.subscribe<TestMessage>("https://goo.gl/DkzYWB", 1, test_callback);
  ASSERT_TRUE(static_cast<bool>(new_s));

  auto new_p = router.advertise<TestMessage>("https://goo.gl/DkzYWB", 1);
  ASSERT_TRUE(static_cast<bool>(new_p));
}


TEST(Router, ScopedAdvertiseThenSubscribe)
{
  flow_ros::Router router{"/router"};

  {
    auto new_p = router.advertise<TestMessage>("https://goo.gl/DkzYWB", 1);
    ASSERT_TRUE(static_cast<bool>(new_p));
  }

  auto new_s = router.subscribe<TestMessage>("https://goo.gl/DkzYWB", 1, test_callback);
  ASSERT_TRUE(static_cast<bool>(new_s));
}


TEST(Router, ScopedSubscribeThenAdvertise)
{
  flow_ros::Router router{"/router"};

  {
    auto new_s = router.subscribe<TestMessage>("https://goo.gl/DkzYWB", 1, test_callback);
    ASSERT_TRUE(static_cast<bool>(new_s));
  }

  auto new_p = router.advertise<TestMessage>("https://goo.gl/DkzYWB", 1);
  ASSERT_TRUE(static_cast<bool>(new_p));
}


TEST(Router, Clear)
{
  flow_ros::Router router{"/router"};
  {
    auto new_s = router.subscribe<TestMessage>("https://goo.gl/DkzYWB", 1, test_callback);
    auto new_p = router.advertise<TestMessage>("https://goo.gl/DkzYWB", 1);
  }

  ASSERT_NO_THROW(router.clear());

  ASSERT_EQ(router.knownSubscriptions().size(), 0u);
  ASSERT_EQ(router.knownPublications().size(), 0u);
}


TEST(Router, CallPublishWithOutOfScopeSubscribers)
{
  flow_ros::Router router{"/router"};
  static const std::string topic = "https://goo.gl/DkzYWB";

  auto obj1 = std::make_shared<CallbackObject>(router, topic);
  auto obj2 = std::make_shared<CallbackObject>(router, topic);
  ASSERT_TRUE(static_cast<bool>(obj1));
  ASSERT_TRUE(static_cast<bool>(obj2));

  ASSERT_TRUE(static_cast<bool>(obj1->subscription_));
  ASSERT_TRUE(static_cast<bool>(obj2->subscription_));

  auto new_p = router.advertise<TestMessage>(topic, 1);
  ASSERT_TRUE(static_cast<bool>(new_p));

  TestMessage::Ptr msg{std::make_shared<TestMessage>()};
  ASSERT_TRUE(static_cast<bool>(msg));

  new_p->publish(msg);

  // After publishing each subscriber should be called once
  EXPECT_EQ(obj1->callback_count_, 1UL);
  EXPECT_EQ(obj2->callback_count_, 1UL);

  // Now release one object
  obj1.reset();
  ASSERT_FALSE(static_cast<bool>(obj1));
  ASSERT_TRUE(static_cast<bool>(obj2));

  // When publishing again, only the second object's callback should be called
  EXPECT_NO_THROW(new_p->publish(msg));
  EXPECT_EQ(obj2->callback_count_, 2UL);
}


TEST(Router, CallPublishWithMultipleSubscribersBeforeSetup)
{
  flow_ros::Router router{"/router"};

  bool result1{false};
  bool result2{false};

  auto new_p = router.advertise<TestMessage>("https://goo.gl/DkzYWB", 1);
  ASSERT_TRUE(static_cast<bool>(new_p));

  // Try and publish
  TestMessage::Ptr msg{std::make_shared<TestMessage>()};
  new_p->publish(msg);

  auto new_s1 = router.subscribe<TestMessage>("https://goo.gl/DkzYWB", 1,
    [&](const TestMessage::ConstPtr& a)
    {
      result1 = true;
    });
  ASSERT_TRUE(static_cast<bool>(new_s1));

  auto new_s2 = router.subscribe<TestMessage>("https://goo.gl/DkzYWB", 1,
    [&](const TestMessage::ConstPtr& b)
    {
      result2 = true;
    });
  ASSERT_TRUE(static_cast<bool>(new_s2));

  EXPECT_FALSE(result1 or result2) << router;
}


TEST(Router, CallPublishWithMultipleSubscribersAfterSetup)
{
  flow_ros::Router router{"/router"};

  bool result1{false};
  bool result2{false};

  auto new_s1 = router.subscribe<TestMessage>("https://goo.gl/DkzYWB", 1,
    [&](const TestMessage::ConstPtr& a)
    {
      result1 = true;
    });
  ASSERT_TRUE(static_cast<bool>(new_s1));

  auto new_s2 = router.subscribe<TestMessage>("https://goo.gl/DkzYWB", 1,
    [&](const TestMessage::ConstPtr& a)
    {
      result2 = true;
    });
  ASSERT_TRUE(static_cast<bool>(new_s2));

  auto new_p = router.advertise<TestMessage>("https://goo.gl/DkzYWB", 1);
  ASSERT_TRUE(static_cast<bool>(new_p));

  // Try and publish
  TestMessage::Ptr msg{std::make_shared<TestMessage>()};
  new_p->publish(msg);

  EXPECT_TRUE(result1 and result2) << router;
}


TEST(Router, CallPublishWithVariedSubscribersAfterSetup)
{
  flow_ros::Router router{"/router"};

  bool result1{false};
  bool result2{false};

  auto new_s1 = router.subscribe<TestMessage>("https://goo.gl/DkzYWB", 1,
    [&](const TestMessage::ConstPtr& a)
    {
      result1 = true;
    });
  ASSERT_TRUE(static_cast<bool>(new_s1));

  auto new_s2 = router.subscribe<TestMessage>("https://goo.gl/EgiRCz", 1,
    [&](const TestMessage::ConstPtr& a)
    {
      result2 = true;
    });
  ASSERT_TRUE(static_cast<bool>(new_s2));

  auto new_p = router.advertise<TestMessage>("https://goo.gl/DkzYWB", 1);
  ASSERT_TRUE(static_cast<bool>(new_p));

  // Try and publish
  TestMessage::Ptr msg{std::make_shared<TestMessage>()};
  new_p->publish(msg);

  EXPECT_TRUE(result1 and !result2) << router;
}


TEST(Router, InjectMessageConstPtrOnTopic)
{
  flow_ros::Router router{"test"};

  const std::string topic{"https://goo.gl/DkzYWB"};

  bool result1{false};

  auto new_s1 = router.subscribe<TestMessage>(topic, 1,
    [&](const TestMessage::ConstPtr&)
    {
      result1 = true;
    });
  ASSERT_TRUE(static_cast<bool>(new_s1));

  auto new_p = router.advertise<TestMessage>(topic, 1);
  ASSERT_TRUE(static_cast<bool>(new_p));

  // Inject a message
  TestMessage::Ptr msg{std::make_shared<TestMessage>()};

  ASSERT_NO_THROW(router.inject(topic, msg)) << router;

  EXPECT_TRUE(result1) << router;
}


TEST(Router, ManyThreadedPubSubSameTopic)
{
  flow_ros::Router router{"test"};

  const std::string topic{"https://goo.gl/DkzYWB"};

  auto new_s1 = router.subscribe<TestMessage>(topic, 1,
    [&](const TestMessage::ConstPtr& msg)
    {
      ASSERT_EQ(msg.use_count(), 2);
    });

  auto new_s2 = router.subscribe<TestMessage>(topic, 1,
    [&](const TestMessage::ConstPtr& msg)
    {
      ASSERT_EQ(msg.use_count(), 2);
    });

  auto new_s3 = router.subscribe<TestMessage>(topic, 1,
    [&](const TestMessage::ConstPtr& msg)
    {
      ASSERT_EQ(msg.use_count(), 2);
    });

  // Create a bunch of publisher threads
  std::vector<std::thread> pub_threads;
  for (std::size_t idx = 0; idx < 4; idx++)
  {
    pub_threads.emplace_back(
      [&]()
      {
        auto pub = router.advertise<TestMessage>(topic, 1);

        std::size_t jdx{0};
        while(++jdx <= 1000)
        {
          TestMessage::Ptr msg{std::make_shared<TestMessage>()};
          pub->publish(msg);
          ASSERT_EQ(msg.use_count(), 1);
        }
      });
  }

  // Join all publisher threads
  for (auto& t : pub_threads)
  {
    t.join();
  }
}
