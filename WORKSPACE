workspace(name = "flow_ros")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")


# GTest and GMock
http_archive(
    name="googletest",
    url="https://github.com/google/googletest/archive/release-1.8.0.zip",
    sha256="f3ed3b58511efd272eb074a3a6d6fb79d7c2e6a0e374323d1e6bcbcc1ef141bf",
    build_file="@flow_ros//:third_party/googletest.BUILD",
    strip_prefix="googletest-release-1.8.0",
)


# ROS (local)
new_local_repository(
  name="ros",
  path="/opt/ros/melodic/",
  build_file="@//:third_party/ros.BUILD",
)


# Boost
new_local_repository(
  name="boost",
  path="/usr/",
  build_file="@//:third_party/boost.BUILD",
)


# Flow (core library)
git_repository(
  name="flow",
  remote="https://github.com/fetchrobotics/flow.git",
  commit="a433c48ee9b2a97538853a182d885187dcf55d9b",
  shallow_since="1604695817 -0500",
)
