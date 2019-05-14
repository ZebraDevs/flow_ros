load("@flow//:flow_rules.bzl", "flow_cc_library")

flow_cc_library(
    name="flow_ros",
    hdrs=glob(["include/**/**/*.h", "include/**/*.hpp"]),
    srcs=glob(["src/**/*.cpp"]),
    strip_include_prefix="//flow_ros/include",
    include_prefix="flow_ros",
    deps=["@flow//flow:flow", "@ros//:roscpp"],
    includes=["include"],
    visibility=["//visibility:public"],
)
