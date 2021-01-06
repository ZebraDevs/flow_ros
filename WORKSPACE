workspace(name = "flow_ros")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@flow_ros//:bazel/ros_build.bzl", "ros_build_generator")


# GTest and GMock
http_archive(
    name="googletest",
    url="https://github.com/google/googletest/archive/release-1.8.0.zip",
    sha256="f3ed3b58511efd272eb074a3a6d6fb79d7c2e6a0e374323d1e6bcbcc1ef141bf",
    build_file="@flow_ros//:bazel/third_party/googletest.BUILD",
    strip_prefix="googletest-release-1.8.0",
)


# ROS (local)
ros_build_generator(name="ros_build")

new_local_repository(
  name="ros",
  path="/opt/ros/",
  build_file="@ros_build//:BUILD",
)

# Boost
new_local_repository(
  name="boost",
  path="/usr/",
  build_file="@//:bazel/third_party/boost.BUILD",
)


# Flow (core library)
git_repository(
  name="flow",
  remote="https://github.com/fetchrobotics/flow.git",
  commit="a433c48ee9b2a97538853a182d885187dcf55d9b",
  shallow_since="1604695817 -0500",
)
