# Reference: to https://github.com/nicolov/boost_bazel
load("@//:utilities/boost.bzl", "boost_library_local")

cc_library(
    name = 'headers',
    visibility = ["//visibility:public"],
    includes = [".",],
    hdrs = glob([
        "boost/**/*.h",
        "boost/**/*.hpp",
        "boost/**/*.ipp",
    ]),
    deps = []
)

boost_library_local(
    name = 'system',
    deps = [':headers',],
)
