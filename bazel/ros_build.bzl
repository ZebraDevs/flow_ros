
def _create_ros_build_file(ros_distro):
  return """
package(default_visibility=['//visibility:public'])

cc_library(
    name='orocos_kdl',
    srcs=glob(['{ros_distro}/lib/liborocos-kdl*.so']),
    hdrs=glob(['{ros_distro}/include/kdl/**/*.h', '{ros_distro}/include/kdl/**/*.hpp', '{ros_distro}/include/kdl/**/*.inl']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':catkin',],
)

cc_library(
    name='cpp_common',
    srcs=glob(['{ros_distro}/lib/libcpp_common*.so']),
    hdrs=glob(['{ros_distro}/include/ros/**/*.h', '{ros_distro}/include/ros/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':catkin', ],
)

cc_library(
    name='catkin',
    srcs=glob(['{ros_distro}/lib/libcatkin*.so']),
    hdrs=glob(['{ros_distro}/include/catkin/**/*.h', '{ros_distro}/include/catkin/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
)

cc_library(
    name='message_generation',
    srcs=glob(['{ros_distro}/lib/libmessage_generation*.so']),
    hdrs=glob(['{ros_distro}/include/message_generation/**/*.h', '{ros_distro}/include/message_generation/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':genlisp', ':genpy', ':genmsg', ':catkin', ':gencpp', ],
)

cc_library(
    name='gencpp',
    srcs=glob(['{ros_distro}/lib/libgencpp*.so']),
    hdrs=glob(['{ros_distro}/include/gencpp/**/*.h', '{ros_distro}/include/gencpp/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':genmsg', ':catkin', ],
)

cc_library(
    name='genmsg',
    srcs=glob(['{ros_distro}/lib/libgenmsg*.so']),
    hdrs=glob(['{ros_distro}/include/genmsg/**/*.h', '{ros_distro}/include/genmsg/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':catkin', ],
)

cc_library(
    name='genpy',
    srcs=glob(['{ros_distro}/lib/libgenpy*.so']),
    hdrs=glob(['{ros_distro}/include/genpy/**/*.h', '{ros_distro}/include/genpy/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':genmsg', ':catkin', ],
)

cc_library(
    name='genlisp',
    srcs=glob(['{ros_distro}/lib/libgenlisp*.so']),
    hdrs=glob(['{ros_distro}/include/genlisp/**/*.h', '{ros_distro}/include/genlisp/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':genmsg', ':catkin', ],
)

cc_library(
    name='roscpp_traits',
    srcs=glob(['{ros_distro}/lib/libroscpp_traits*.so']),
    hdrs=glob(['{ros_distro}/include/roscpp_traits/**/*.h', '{ros_distro}/include/roscpp_traits/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':rostime', ':cpp_common', ':catkin', ],
)

cc_library(
    name='rostime',
    srcs=glob(['{ros_distro}/lib/librostime*.so']),
    hdrs=glob(['{ros_distro}/include/rostime/**/*.h', '{ros_distro}/include/rostime/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':cpp_common', ':catkin', ],
)

cc_library(
    name='roscpp_serialization',
    srcs=glob(['{ros_distro}/lib/libroscpp_serialization*.so']),
    hdrs=glob(['{ros_distro}/include/roscpp_serialization/**/*.h', '{ros_distro}/include/roscpp_serialization/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':roscpp_traits', ':catkin', ':cpp_common', ':rostime', ],
)

cc_library(
    name='message_runtime',
    srcs=glob(['{ros_distro}/lib/libmessage_runtime*.so']),
    hdrs=glob(['{ros_distro}/include/message_runtime/**/*.h', '{ros_distro}/include/message_runtime/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':genpy', ':genmsg', ':catkin', ':rostime', ':roscpp_serialization', ':roscpp_traits', ':cpp_common', ],
)

cc_library(
    name='std_msgs',
    srcs=glob(['{ros_distro}/lib/libstd_msgs*.so']),
    hdrs=glob(['{ros_distro}/include/std_msgs/**/*.h', '{ros_distro}/include/std_msgs/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':genlisp', ':genpy', ':genmsg', ':catkin', ':gencpp', ':message_runtime', ':rostime', ':roscpp_serialization', ':roscpp_traits', ':message_generation', ':cpp_common', ],
)

cc_library(
    name='rosbag',
    srcs=glob(['{ros_distro}/lib/librosbag*.so']),
    hdrs=glob(['{ros_distro}/include/rosbag/**/*.h', '{ros_distro}/include/rosbag/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':rosconsole', ':rosparam', ':catkin', ':rosgraph', ':cmake_modules', ':rospack', ':rosbuild', ':roscpp', ':gencpp', ':roslib', ':topic_tools', ':roscpp_serialization', ':roscpp_traits', ':rosout', ':rostest', ':roslaunch', ':message_generation', ':roslang', ':roslz4', ':genlisp', ':genpy', ':rostime', ':message_runtime', ':std_msgs', ':rospy', ':rosunit', ':rosbag_storage', ':rosgraph_msgs', ':genmsg', ':xmlrpcpp', ':cpp_common', ':rosmaster', ':rosclean', ],
)

cc_library(
    name='rosclean',
    srcs=glob(['{ros_distro}/lib/librosclean*.so']),
    hdrs=glob(['{ros_distro}/include/rosclean/**/*.h', '{ros_distro}/include/rosclean/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':catkin', ],
)

cc_library(
    name='rosmaster',
    srcs=glob(['{ros_distro}/lib/librosmaster*.so']),
    hdrs=glob(['{ros_distro}/include/rosmaster/**/*.h', '{ros_distro}/include/rosmaster/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':rosgraph', ':catkin', ],
)

cc_library(
    name='rosgraph',
    srcs=glob(['{ros_distro}/lib/librosgraph*.so']),
    hdrs=glob(['{ros_distro}/include/rosgraph/**/*.h', '{ros_distro}/include/rosgraph/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':catkin', ],
)

cc_library(
    name='xmlrpcpp',
    srcs=glob(['{ros_distro}/lib/libxmlrpcpp*.so']),
    hdrs=glob(['{ros_distro}/include/*.h', '{ros_distro}/include/*.hpp', '{ros_distro}/include/xmlrpcpp/*.h']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':cpp_common', ':catkin', ],
)

cc_library(
    name='rosgraph_msgs',
    srcs=glob(['{ros_distro}/lib/librosgraph_msgs*.so']),
    hdrs=glob(['{ros_distro}/include/rosgraph_msgs/**/*.h', '{ros_distro}/include/rosgraph_msgs/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':genlisp', ':genpy', ':genmsg', ':catkin', ':gencpp', ':std_msgs', ':message_runtime', ':rostime', ':roscpp_serialization', ':roscpp_traits', ':message_generation', ':cpp_common', ],
)

cc_library(
    name='rosbag_storage',
    srcs=glob(['{ros_distro}/lib/librosbag_storage*.so']),
    hdrs=glob(['{ros_distro}/include/rosbag_storage/**/*.h', '{ros_distro}/include/rosbag_storage/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':roslz4', ':catkin', ':roslib', ':rostime', ':roscpp_serialization', ':roscpp_traits', ':cmake_modules', ':rosunit', ':rospack', ':cpp_common', 'pluginlib', ],
)

cc_library(
    name='pluginlib',
    hdrs=glob(['{ros_distro}/include/pluginlib/*.h', '{ros_distro}/include/pluginlib/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':roslib', ':rosconsole', ':class_loader'],
)

cc_library(
    name='class_loader',
    srcs=['{ros_distro}/lib/libclass_loader.so'],
    hdrs=glob(['{ros_distro}/include/class_loader/**/*.h', '{ros_distro}/include/class_loader/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':catkin', ':cmake_modules'],
)

cc_library(
    name='rospack',
    srcs=glob(['{ros_distro}/lib/librospack*.so']),
    hdrs=glob(['{ros_distro}/include/rospack/**/*.h', '{ros_distro}/include/rospack/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':cmake_modules', ':catkin', ],
)

cc_library(
    name='cmake_modules',
    srcs=glob(['{ros_distro}/lib/libcmake_modules*.so']),
    hdrs=glob(['{ros_distro}/include/cmake_modules/**/*.h', '{ros_distro}/include/cmake_modules/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':catkin', ],
)

cc_library(
    name='rosunit',
    srcs=glob(['{ros_distro}/lib/librosunit*.so']),
    hdrs=glob(['{ros_distro}/include/rosunit/**/*.h', '{ros_distro}/include/rosunit/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':catkin', ':cmake_modules', ':roslib', ':rospack', ],
)

cc_library(
    name='roslib',
    srcs=glob(['{ros_distro}/lib/libroslib*.so']),
    hdrs=glob(['{ros_distro}/include/ros{ros_distro}/lib/**/*.h', '{ros_distro}/include/ros{ros_distro}/lib/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':rospack', ':cmake_modules', ':catkin', ],
)

cc_library(
    name='roslz4',
    srcs=glob(['{ros_distro}/lib/libroslz4*.so']),
    hdrs=glob(['{ros_distro}/include/roslz4/**/*.h', '{ros_distro}/include/roslz4/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':rospack', ':cmake_modules', ':roslib', ':catkin', ':rosunit', ],
)

cc_library(
    name='rospy',
    srcs=glob(['{ros_distro}/lib/librospy*.so']),
    hdrs=glob(['{ros_distro}/include/rospy/**/*.h', '{ros_distro}/include/rospy/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':rosconsole', ':catkin', ':rosgraph', ':cmake_modules', ':rospack', ':rosbuild', ':roscpp', ':gencpp', ':roslib', ':roscpp_serialization', ':roscpp_traits', ':message_generation', ':roslang', ':genlisp', ':genpy', ':std_msgs', ':message_runtime', ':rostime', ':rosunit', ':rosgraph_msgs', ':genmsg', ':xmlrpcpp', ':cpp_common', ],
)

cc_library(
    name='roslang',
    srcs=glob(['{ros_distro}/lib/libroslang*.so']),
    hdrs=glob(['{ros_distro}/include/roslang/**/*.h', '{ros_distro}/include/roslang/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':genmsg', ':catkin', ],
)

cc_library(
    name='roscpp',
    srcs=glob(['{ros_distro}/lib/libroscpp*.so']),
    hdrs=glob(['{ros_distro}/include/roscpp/**/*.h', '{ros_distro}/include/roscpp/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':rosgraph_msgs', ':genlisp', ':genpy', ':genmsg', ':catkin', ':rosconsole', ':gencpp', ':xmlrpcpp', ':rostime', ':message_runtime', ':std_msgs', ':roslang', ':roscpp_serialization', ':roscpp_traits', ':roslib', ':cmake_modules', ':rosunit', ':rosbuild', ':message_generation', ':rospack', ':cpp_common', ],
)

cc_library(
    name='rosbuild',
    srcs=glob(['{ros_distro}/lib/librosbuild*.so']),
    hdrs=glob(['{ros_distro}/include/rosbuild/**/*.h', '{ros_distro}/include/rosbuild/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':genlisp', ':genpy', ':genmsg', ':catkin', ':gencpp', ':message_runtime', ':rostime', ':roscpp_serialization', ':roscpp_traits', ':message_generation', ':cpp_common', ],
)

cc_library(
    name='rosconsole',
    srcs=glob(['{ros_distro}/lib/librosconsole*.so']),
    hdrs=glob(['{ros_distro}/include/rosconsole/**/*.h', '{ros_distro}/include/rosconsole/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':genlisp', ':genpy', ':genmsg', ':catkin', ':gencpp', ':message_runtime', ':rostime', ':roscpp_serialization', ':roscpp_traits', ':roslib', ':cmake_modules', ':rosunit', ':rosbuild', ':message_generation', ':rospack', ':cpp_common'],
)

cc_library(
    name='roslaunch',
    srcs=glob(['{ros_distro}/lib/libroslaunch*.so']),
    hdrs=glob(['{ros_distro}/include/roslaunch/**/*.h', '{ros_distro}/include/roslaunch/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':rosconsole', ':rosparam', ':catkin', ':rosgraph', ':cmake_modules', ':rospack', ':rosbuild', ':roscpp', ':gencpp', ':roslib', ':roscpp_serialization', ':roscpp_traits', ':rosout', ':rosclean', ':message_generation', ':roslang', ':genlisp', ':genpy', ':std_msgs', ':message_runtime', ':rostime', ':rosunit', ':rosgraph_msgs', ':genmsg', ':xmlrpcpp', ':cpp_common', ':rosmaster', ],
)

cc_library(
    name='rosout',
    srcs=glob(['{ros_distro}/lib/librosout*.so']),
    hdrs=glob(['{ros_distro}/include/rosout/**/*.h', '{ros_distro}/include/rosout/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':rosconsole', ':catkin', ':cmake_modules', ':rospack', ':rosbuild', ':roscpp', ':gencpp', ':roslib', ':roscpp_serialization', ':roscpp_traits', ':message_generation', ':roslang', ':genlisp', ':genpy', ':std_msgs', ':message_runtime', ':rostime', ':rosunit', ':rosgraph_msgs', ':genmsg', ':xmlrpcpp', ':cpp_common', ],
)

cc_library(
    name='rosparam',
    srcs=glob(['{ros_distro}/lib/librosparam*.so']),
    hdrs=glob(['{ros_distro}/include/rosparam/**/*.h', '{ros_distro}/include/rosparam/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':rosgraph', ':catkin', ],
)

cc_library(
    name='rostest',
    srcs=glob(['{ros_distro}/lib/librostest*.so']),
    hdrs=glob(['{ros_distro}/include/rostest/**/*.h', '{ros_distro}/include/rostest/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':rosconsole', ':rosparam', ':catkin', ':rosgraph', ':cmake_modules', ':rospack', ':roscpp_traits', ':roscpp', ':gencpp', ':roslib', ':roscpp_serialization', ':rosbuild', ':rosout', ':rosclean', ':roslaunch', ':message_generation', ':roslang', ':genlisp', ':genpy', ':rostime', ':message_runtime', ':std_msgs', ':rospy', ':rosunit', ':rosgraph_msgs', ':genmsg', ':xmlrpcpp', ':cpp_common', ':rosmaster', ],
)

cc_library(
    name='topic_tools',
    srcs=glob(['{ros_distro}/lib/libtopic_tools*.so']),
    hdrs=glob(['{ros_distro}/include/topic_tools/**/*.h', '{ros_distro}/include/topic_tools/**/*.hpp']),
    strip_include_prefix='{ros_distro}/include',
    deps=[':rosconsole', ':roslib', ':catkin', ':rostest', ':rosgraph', ':cmake_modules', ':rospack', ':rosbuild', ':roscpp', ':gencpp', ':rosparam', ':roscpp_serialization', ':roscpp_traits', ':rosout', ':rosclean', ':roslaunch', ':message_generation', ':roslang', ':genlisp', ':genpy', ':rostime', ':message_runtime', ':std_msgs', ':rospy', ':rosunit', ':rosgraph_msgs', ':genmsg', ':xmlrpcpp', ':cpp_common', ':rosmaster', ],
)
""".format(ros_distro=ros_distro)


def _ros_build_generator_impl(repository_ctx):
    ros_distro = repository_ctx.os.environ["ROS_DISTRO"]
    print("Using ROS_DISTRO={}".format(ros_distro))
    repository_ctx.file("BUILD", _create_ros_build_file(ros_distro=ros_distro))


ros_build_generator = repository_rule(
    implementation=_ros_build_generator_impl,
    local = False,
    environ = ["ROS_DISTRO"]
)
