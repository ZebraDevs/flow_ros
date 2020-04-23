/**
 * @copyright 2020 Fetch Robotics Inc.
 * @author Brian Cairl
 *
 * @file  event_handler.cpp
 */

// C++ Standard Library
#include <string>
#include <thread>

// GTest
#include <gtest/gtest.h>

// ROS Messages
#include <std_msgs/Header.h>

// Flow
#include <flow/flow.h>
#include <flow_ros/event_handler.h>
#include <flow_ros/event_handler_ostream.h>
#include <flow_ros/publisher.h>
#include <flow_ros/subscriber.h>


/// A ROS messages-like object
struct TestMessage1
{
  using Ptr = std::shared_ptr<TestMessage1>;
  using ConstPtr = std::shared_ptr<const TestMessage1>;

  std_msgs::Header header;
};


struct TestMessage2
{
  using Ptr = std::shared_ptr<TestMessage2>;
  using ConstPtr = std::shared_ptr<const TestMessage2>;

  std_msgs::Header header;
};


TEST(EventHandlerSingleThreaded, GetSubscribers)
{
  flow_ros::Router router{"/router"};

  // SISO event handler type
  using EventHandler = flow_ros::EventHandler<
    std::tuple<flow_ros::Publisher<TestMessage1>>,
    std::tuple<flow_ros::Subscriber<TestMessage1, flow::driver::Next, flow::NoLock>>
  >;

  // Dummy callback; returns 1 value
  const auto callback =
    [](const EventHandler::Input& inputs)
    {
      return std::make_shared<TestMessage1>();
    };

  // Event handler
  std::shared_ptr<flow_ros::EventHandlerBase> handler =
    std::make_shared<EventHandler>(
      EventHandler::Callbacks{callback},
      std::make_tuple(std::make_shared<flow_ros::Publisher<TestMessage1>>(router, "output", 1)),
      std::make_tuple(std::make_shared<flow_ros::Subscriber<TestMessage1, flow::driver::Next, flow::NoLock>>(router, "input", 1))
    );

  // Get subscribers
  const auto subs = handler->getSubscribers();
  ASSERT_EQ(subs.size(), 1UL);
  ASSERT_EQ(subs.front()->getTopic(), "/router/input");
}


TEST(EventHandlerSingleThreaded, GetPublishers)
{
  flow_ros::Router router{"/router"};

  // SISO event handler type
  using EventHandler = flow_ros::EventHandler<
    std::tuple<flow_ros::Publisher<TestMessage1>>,
    std::tuple<flow_ros::Subscriber<TestMessage1, flow::driver::Next, flow::NoLock>>
  >;

  // Dummy callback; returns 1 value
  const auto callback =
    [](const EventHandler::Input& inputs)
    {
      return std::make_shared<TestMessage1>();
    };

  // Event handler
  std::shared_ptr<flow_ros::EventHandlerBase> handler =
    std::make_shared<EventHandler>(
      EventHandler::Callbacks{callback},
      std::make_tuple(std::make_shared<flow_ros::Publisher<TestMessage1>>(router, "output", 1)),
      std::make_tuple(std::make_shared<flow_ros::Subscriber<TestMessage1, flow::driver::Next, flow::NoLock>>(router, "input", 1))
    );

  // Get publishers
  const auto pubs = handler->getPublishers();
  ASSERT_EQ(pubs.size(), 1UL);
  ASSERT_EQ(pubs.front()->getTopic(), "/router/output");
}


TEST(EventHandlerSingleThreaded, SingleInputNoOutputs)
{
  flow_ros::Router router{"/router"};

  // SISO event handler type
  using EventHandler = flow_ros::EventHandler<
    std::tuple<>,
    std::tuple<flow_ros::Subscriber<TestMessage1, flow::driver::Next, flow::NoLock>>
  >;

  // Publisher to send message
  flow_ros::Publisher<TestMessage1> input_pub{router, "input", 1};

  // Dummy callback; returns 1 value
  const auto callback =
    [](const EventHandler::Input& inputs) -> std::tuple<>
    {
      return std::tuple<>{};
    };

  // Event handler
  std::shared_ptr<flow_ros::EventHandlerBase> handler =
    std::make_shared<EventHandler>(
      EventHandler::Callbacks{callback},
      std::make_tuple(),
      std::make_tuple(std::make_shared<flow_ros::Subscriber<TestMessage1, flow::driver::Next, flow::NoLock>>(router, "input", 1))
    );

  ASSERT_EQ(handler->update().state, flow_ros::EventSummary::State::SYNC_NEEDS_RETRY);

  input_pub.publish(
    []()
    {
      auto msg = std::make_shared<TestMessage1>();
      msg->header.stamp.fromSec(1);
      return msg;
    }()
  );

  ASSERT_EQ(handler->update().state, flow_ros::EventSummary::State::EXECUTED);
}


TEST(EventHandlerSingleThreaded, SingleInputSingleOutput)
{
  flow_ros::Router router{"/router"};

  // SISO event handler type
  using EventHandler = flow_ros::EventHandler<
    std::tuple<flow_ros::Publisher<TestMessage1>>,
    std::tuple<flow_ros::Subscriber<TestMessage1, flow::driver::Next, flow::NoLock>>
  >;

  // Publisher to send message
  flow_ros::Publisher<TestMessage1> input_pub{router, "input", 1};

  // Output subscriber
  TestMessage1::ConstPtr output_msg;
  auto output_sub = router.subscribe<TestMessage1>(
    "output", 1,
    [&output_msg](const TestMessage1::ConstPtr& msg)
    {
      output_msg = msg;
    });

  // Dummy callback; returns 1 value
  const auto callback =
    [](const EventHandler::Input& inputs) -> TestMessage1::Ptr
    {
      return std::make_shared<TestMessage1>();
    };

  // Event handler
  std::shared_ptr<flow_ros::EventHandlerBase> handler =
    std::make_shared<EventHandler>(
      EventHandler::Callbacks{callback},
      std::make_tuple(std::make_shared<flow_ros::Publisher<TestMessage1>>(router, "output", 1)),
      std::make_tuple(std::make_shared<flow_ros::Subscriber<TestMessage1, flow::driver::Next, flow::NoLock>>(router, "input", 1))
    );

  ASSERT_FALSE(output_msg);
  ASSERT_EQ(handler->update().state, flow_ros::EventSummary::State::SYNC_NEEDS_RETRY);
  ASSERT_FALSE(output_msg);

  input_pub.publish(
    []()
    {
      auto msg = std::make_shared<TestMessage1>();
      msg->header.stamp.fromSec(1);
      return msg;
    }()
  );

  ASSERT_EQ(handler->update().state, flow_ros::EventSummary::State::EXECUTED);
  ASSERT_TRUE(output_msg);
  ASSERT_EQ(output_msg->header.stamp, ros::Time{1});
}


TEST(EventHandlerSingleThreaded, SingleInputMultiOutput)
{
  flow_ros::Router router{"/router"};

  // SISO event handler type
  using EventHandler = flow_ros::EventHandler<
    std::tuple<flow_ros::Publisher<TestMessage1>,
               flow_ros::Publisher<TestMessage2>>,
    std::tuple<flow_ros::Subscriber<TestMessage1, flow::driver::Next, flow::NoLock>>
  >;

  // Publisher to send message
  flow_ros::Publisher<TestMessage1> input_pub{router, "input", 1};

  // Output subscribers
  TestMessage1::ConstPtr output_msg_1;
  auto output_sub_1 = router.subscribe<TestMessage1>(
    "output_1", 1,
    [&output_msg_1](const TestMessage1::ConstPtr& msg)
    {
      output_msg_1 = msg;
    });

  TestMessage2::ConstPtr output_msg_2;
  auto output_sub_2 = router.subscribe<TestMessage2>(
    "output_2", 1,
    [&output_msg_2](const TestMessage2::ConstPtr& msg)
    {
      output_msg_2 = msg;
    });

  // Dummy callback; returns 2 values
  const auto callback =
    [](const EventHandler::Input& inputs)
    {
      return std::make_tuple(std::make_shared<TestMessage1>(), std::make_shared<TestMessage2>());
    };

  // Event handler
  std::shared_ptr<flow_ros::EventHandlerBase> handler =
    std::make_shared<EventHandler>(
      callback,
      std::make_tuple(std::make_shared<flow_ros::Publisher<TestMessage1>>(router, "output_1", 1),
                      std::make_shared<flow_ros::Publisher<TestMessage2>>(router, "output_2", 1)),
      std::make_tuple(std::make_shared<flow_ros::Subscriber<TestMessage1, flow::driver::Next, flow::NoLock>>(router, "input", 1))
    );

  ASSERT_FALSE(output_msg_1);
  ASSERT_FALSE(output_msg_2);
  ASSERT_EQ(handler->update().state, flow_ros::EventSummary::State::SYNC_NEEDS_RETRY);

  ASSERT_FALSE(output_msg_1);
  ASSERT_FALSE(output_msg_2);

  input_pub.publish(
    []()
    {
      auto msg = std::make_shared<TestMessage1>();
      msg->header.stamp.fromSec(1);
      return msg;
    }()
  );

  ASSERT_EQ(handler->update().state, flow_ros::EventSummary::State::EXECUTED);

  ASSERT_TRUE(output_msg_1);
  ASSERT_TRUE(output_msg_2);
  ASSERT_EQ(output_msg_1->header.stamp, ros::Time{1});
  ASSERT_EQ(output_msg_2->header.stamp, ros::Time{1});
}


TEST(EventHandlerSingleThreaded, SingleInputOutputArray)
{
  flow_ros::Router router{"/router"};

  // SISO event handler type
  using EventHandler = flow_ros::EventHandler<
    std::tuple<flow_ros::MultiPublisher<TestMessage1>>,
    std::tuple<flow_ros::Subscriber<TestMessage1, flow::driver::Next, flow::NoLock>>
  >;

  // Publisher to send message
  flow_ros::Publisher<TestMessage1> input_pub{router, "input", 1};

  // Output subscribers
  TestMessage1::ConstPtr output_msg_1;
  auto output_sub_1 = router.subscribe<TestMessage1>(
    "output_1", 1,
    [&output_msg_1](const TestMessage1::ConstPtr& msg)
    {
      output_msg_1 = msg;
    });

  // Dummy callback; returns 2 values
  const auto callback =
    [](const EventHandler::Input& inputs)
    {
      std::vector<TestMessage1::Ptr> out_arr{std::make_shared<TestMessage1>()};
      return std::make_tuple(out_arr);
    };

  // Event handler
  std::shared_ptr<flow_ros::EventHandlerBase> handler =
    std::make_shared<EventHandler>(
      callback,
      std::make_tuple(std::make_shared<flow_ros::MultiPublisher<TestMessage1>>(router, "output_1", 1)),
      std::make_tuple(std::make_shared<flow_ros::Subscriber<TestMessage1, flow::driver::Next, flow::NoLock>>(router, "input", 1))
    );

  ASSERT_FALSE(output_msg_1);
  ASSERT_EQ(handler->update().state, flow_ros::EventSummary::State::SYNC_NEEDS_RETRY);

  ASSERT_FALSE(output_msg_1);

  input_pub.publish(
    []()
    {
      auto msg = std::make_shared<TestMessage1>();
      msg->header.stamp.fromSec(1);
      return msg;
    }()
  );

  const auto result = handler->update();
  ASSERT_EQ(result.state, flow_ros::EventSummary::State::EXECUTED) << result;

  ASSERT_TRUE(output_msg_1);
  ASSERT_EQ(output_msg_1->header.stamp, ros::Time{1});
}
