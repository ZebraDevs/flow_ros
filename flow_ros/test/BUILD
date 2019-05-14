load("@flow//:flow_rules.bzl", "flow_cc_gtest")

flow_cc_gtest(
    name="router",
    srcs=["unit/router.cpp"],
    deps=["//flow_ros:flow_ros",],
    timeout="short",
)


flow_cc_gtest(
    name="subscriber",
    srcs=["unit/subscriber.cpp"],
    deps=[
        "//flow_ros:flow_ros",
        "@ros//:std_msgs"
    ],
    timeout="short",
)


flow_cc_gtest(
    name="publisher",
    srcs=["unit/publisher.cpp"],
    deps=[
        "//flow_ros:flow_ros",
        "@ros//:std_msgs"
    ],
    timeout="short",
)


flow_cc_gtest(
    name="event_handler",
    srcs=["unit/event_handler.cpp"],
    deps=[
        "//flow_ros:flow_ros",
        "@ros//:std_msgs"
    ],
    timeout="short",
)

