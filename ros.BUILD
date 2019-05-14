package(default_visibility=['//visibility:public'])


cc_library(
    name='orocos_kdl',
    srcs=glob(['lib/liborocos-kdl*.so']),
    hdrs=glob(['include/kdl/**/*.h', 'include/kdl/**/*.hpp', 'include/kdl/**/*.inl']),
    strip_include_prefix='include',
    deps=[':catkin',],
)

cc_library(
    name='cpp_common',
    srcs=glob(['lib/libcpp_common*.so']),
    hdrs=glob(['include/ros/**/*.h', 'include/ros/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':catkin', ],
)

cc_library(
    name='catkin',
    srcs=glob(['lib/libcatkin*.so']),
    hdrs=glob(['include/catkin/**/*.h', 'include/catkin/**/*.hpp']),
    strip_include_prefix='include',
)

cc_library(
    name='message_generation',
    srcs=glob(['lib/libmessage_generation*.so']),
    hdrs=glob(['include/message_generation/**/*.h', 'include/message_generation/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':genlisp', ':genpy', ':genmsg', ':catkin', ':gencpp', ],
)

cc_library(
    name='gencpp',
    srcs=glob(['lib/libgencpp*.so']),
    hdrs=glob(['include/gencpp/**/*.h', 'include/gencpp/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':genmsg', ':catkin', ],
)

cc_library(
    name='genmsg',
    srcs=glob(['lib/libgenmsg*.so']),
    hdrs=glob(['include/genmsg/**/*.h', 'include/genmsg/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':catkin', ],
)

cc_library(
    name='genpy',
    srcs=glob(['lib/libgenpy*.so']),
    hdrs=glob(['include/genpy/**/*.h', 'include/genpy/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':genmsg', ':catkin', ],
)

cc_library(
    name='genlisp',
    srcs=glob(['lib/libgenlisp*.so']),
    hdrs=glob(['include/genlisp/**/*.h', 'include/genlisp/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':genmsg', ':catkin', ],
)

cc_library(
    name='roscpp_traits',
    srcs=glob(['lib/libroscpp_traits*.so']),
    hdrs=glob(['include/roscpp_traits/**/*.h', 'include/roscpp_traits/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':rostime', ':cpp_common', ':catkin', ],
)

cc_library(
    name='rostime',
    srcs=glob(['lib/librostime*.so']),
    hdrs=glob(['include/rostime/**/*.h', 'include/rostime/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':cpp_common', ':catkin', ],
)

cc_library(
    name='roscpp_serialization',
    srcs=glob(['lib/libroscpp_serialization*.so']),
    hdrs=glob(['include/roscpp_serialization/**/*.h', 'include/roscpp_serialization/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':roscpp_traits', ':catkin', ':cpp_common', ':rostime', ],
)

cc_library(
    name='message_runtime',
    srcs=glob(['lib/libmessage_runtime*.so']),
    hdrs=glob(['include/message_runtime/**/*.h', 'include/message_runtime/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':genpy', ':genmsg', ':catkin', ':rostime', ':roscpp_serialization', ':roscpp_traits', ':cpp_common', ],
)

cc_library(
    name='std_msgs',
    srcs=glob(['lib/libstd_msgs*.so']),
    hdrs=glob(['include/std_msgs/**/*.h', 'include/std_msgs/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':genlisp', ':genpy', ':genmsg', ':catkin', ':gencpp', ':message_runtime', ':rostime', ':roscpp_serialization', ':roscpp_traits', ':message_generation', ':cpp_common', ],
)

cc_library(
    name='rosbag',
    srcs=glob(['lib/librosbag*.so']),
    hdrs=glob(['include/rosbag/**/*.h', 'include/rosbag/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':rosconsole', ':rosparam', ':catkin', ':rosgraph', ':cmake_modules', ':rospack', ':rosbuild', ':roscpp', ':gencpp', ':roslib', ':topic_tools', ':roscpp_serialization', ':roscpp_traits', ':rosout', ':rostest', ':roslaunch', ':message_generation', ':roslang', ':roslz4', ':genlisp', ':genpy', ':rostime', ':message_runtime', ':std_msgs', ':rospy', ':rosunit', ':rosbag_storage', ':rosgraph_msgs', ':genmsg', ':xmlrpcpp', ':cpp_common', ':rosmaster', ':rosclean', ],
)

cc_library(
    name='rosclean',
    srcs=glob(['lib/librosclean*.so']),
    hdrs=glob(['include/rosclean/**/*.h', 'include/rosclean/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':catkin', ],
)

cc_library(
    name='rosmaster',
    srcs=glob(['lib/librosmaster*.so']),
    hdrs=glob(['include/rosmaster/**/*.h', 'include/rosmaster/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':rosgraph', ':catkin', ],
)

cc_library(
    name='rosgraph',
    srcs=glob(['lib/librosgraph*.so']),
    hdrs=glob(['include/rosgraph/**/*.h', 'include/rosgraph/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':catkin', ],
)

cc_library(
    name='xmlrpcpp',
    srcs=glob(['lib/libxmlrpcpp*.so']),
    hdrs=glob(['include/*.h', 'include/*.hpp', 'include/xmlrpcpp/*.h']),
    strip_include_prefix='include',
    deps=[':cpp_common', ':catkin', ],
)

cc_library(
    name='rosgraph_msgs',
    srcs=glob(['lib/librosgraph_msgs*.so']),
    hdrs=glob(['include/rosgraph_msgs/**/*.h', 'include/rosgraph_msgs/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':genlisp', ':genpy', ':genmsg', ':catkin', ':gencpp', ':std_msgs', ':message_runtime', ':rostime', ':roscpp_serialization', ':roscpp_traits', ':message_generation', ':cpp_common', ],
)

cc_library(
    name='rosbag_storage',
    srcs=glob(['lib/librosbag_storage*.so']),
    hdrs=glob(['include/rosbag_storage/**/*.h', 'include/rosbag_storage/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':roslz4', ':catkin', ':roslib', ':rostime', ':roscpp_serialization', ':roscpp_traits', ':cmake_modules', ':rosunit', ':rospack', ':cpp_common', 'pluginlib', ],
)

cc_library(
    name='pluginlib',
    hdrs=glob(['include/pluginlib/**/*.h', 'include/pluginlib/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':roslib', ':rosconsole', ':class_loader'],
)

cc_library(
    name='class_loader',
    hdrs=glob(['include/class_loader/**/*.h', 'include/class_loader/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':catkin', ':cmake_modules'],
)

cc_library(
    name='rospack',
    srcs=glob(['lib/librospack*.so']),
    hdrs=glob(['include/rospack/**/*.h', 'include/rospack/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':cmake_modules', ':catkin', ],
)

cc_library(
    name='cmake_modules',
    srcs=glob(['lib/libcmake_modules*.so']),
    hdrs=glob(['include/cmake_modules/**/*.h', 'include/cmake_modules/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':catkin', ],
)

cc_library(
    name='rosunit',
    srcs=glob(['lib/librosunit*.so']),
    hdrs=glob(['include/rosunit/**/*.h', 'include/rosunit/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':catkin', ':cmake_modules', ':roslib', ':rospack', ],
)

cc_library(
    name='roslib',
    srcs=glob(['lib/libroslib*.so']),
    hdrs=glob(['include/roslib/**/*.h', 'include/roslib/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':rospack', ':cmake_modules', ':catkin', ],
)

cc_library(
    name='roslz4',
    srcs=glob(['lib/libroslz4*.so']),
    hdrs=glob(['include/roslz4/**/*.h', 'include/roslz4/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':rospack', ':cmake_modules', ':roslib', ':catkin', ':rosunit', ],
)

cc_library(
    name='rospy',
    srcs=glob(['lib/librospy*.so']),
    hdrs=glob(['include/rospy/**/*.h', 'include/rospy/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':rosconsole', ':catkin', ':rosgraph', ':cmake_modules', ':rospack', ':rosbuild', ':roscpp', ':gencpp', ':roslib', ':roscpp_serialization', ':roscpp_traits', ':message_generation', ':roslang', ':genlisp', ':genpy', ':std_msgs', ':message_runtime', ':rostime', ':rosunit', ':rosgraph_msgs', ':genmsg', ':xmlrpcpp', ':cpp_common', ],
)

cc_library(
    name='roslang',
    srcs=glob(['lib/libroslang*.so']),
    hdrs=glob(['include/roslang/**/*.h', 'include/roslang/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':genmsg', ':catkin', ],
)

cc_library(
    name='roscpp',
    srcs=glob(['lib/libroscpp*.so']),
    hdrs=glob(['include/roscpp/**/*.h', 'include/roscpp/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':rosgraph_msgs', ':genlisp', ':genpy', ':genmsg', ':catkin', ':rosconsole', ':gencpp', ':xmlrpcpp', ':rostime', ':message_runtime', ':std_msgs', ':roslang', ':roscpp_serialization', ':roscpp_traits', ':roslib', ':cmake_modules', ':rosunit', ':rosbuild', ':message_generation', ':rospack', ':cpp_common', ],
)

cc_library(
    name='rosbuild',
    srcs=glob(['lib/librosbuild*.so']),
    hdrs=glob(['include/rosbuild/**/*.h', 'include/rosbuild/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':genlisp', ':genpy', ':genmsg', ':catkin', ':gencpp', ':message_runtime', ':rostime', ':roscpp_serialization', ':roscpp_traits', ':message_generation', ':cpp_common', ],
)

cc_library(
    name='rosconsole',
    srcs=glob(['lib/librosconsole*.so']),
    hdrs=glob(['include/rosconsole/**/*.h', 'include/rosconsole/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':genlisp', ':genpy', ':genmsg', ':catkin', ':gencpp', ':message_runtime', ':rostime', ':roscpp_serialization', ':roscpp_traits', ':roslib', ':cmake_modules', ':rosunit', ':rosbuild', ':message_generation', ':rospack', ':cpp_common'],
)

cc_library(
    name='roslaunch',
    srcs=glob(['lib/libroslaunch*.so']),
    hdrs=glob(['include/roslaunch/**/*.h', 'include/roslaunch/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':rosconsole', ':rosparam', ':catkin', ':rosgraph', ':cmake_modules', ':rospack', ':rosbuild', ':roscpp', ':gencpp', ':roslib', ':roscpp_serialization', ':roscpp_traits', ':rosout', ':rosclean', ':message_generation', ':roslang', ':genlisp', ':genpy', ':std_msgs', ':message_runtime', ':rostime', ':rosunit', ':rosgraph_msgs', ':genmsg', ':xmlrpcpp', ':cpp_common', ':rosmaster', ],
)

cc_library(
    name='rosout',
    srcs=glob(['lib/librosout*.so']),
    hdrs=glob(['include/rosout/**/*.h', 'include/rosout/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':rosconsole', ':catkin', ':cmake_modules', ':rospack', ':rosbuild', ':roscpp', ':gencpp', ':roslib', ':roscpp_serialization', ':roscpp_traits', ':message_generation', ':roslang', ':genlisp', ':genpy', ':std_msgs', ':message_runtime', ':rostime', ':rosunit', ':rosgraph_msgs', ':genmsg', ':xmlrpcpp', ':cpp_common', ],
)

cc_library(
    name='rosparam',
    srcs=glob(['lib/librosparam*.so']),
    hdrs=glob(['include/rosparam/**/*.h', 'include/rosparam/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':rosgraph', ':catkin', ],
)

cc_library(
    name='rostest',
    srcs=glob(['lib/librostest*.so']),
    hdrs=glob(['include/rostest/**/*.h', 'include/rostest/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':rosconsole', ':rosparam', ':catkin', ':rosgraph', ':cmake_modules', ':rospack', ':roscpp_traits', ':roscpp', ':gencpp', ':roslib', ':roscpp_serialization', ':rosbuild', ':rosout', ':rosclean', ':roslaunch', ':message_generation', ':roslang', ':genlisp', ':genpy', ':rostime', ':message_runtime', ':std_msgs', ':rospy', ':rosunit', ':rosgraph_msgs', ':genmsg', ':xmlrpcpp', ':cpp_common', ':rosmaster', ],
)

cc_library(
    name='topic_tools',
    srcs=glob(['lib/libtopic_tools*.so']),
    hdrs=glob(['include/topic_tools/**/*.h', 'include/topic_tools/**/*.hpp']),
    strip_include_prefix='include',
    deps=[':rosconsole', ':roslib', ':catkin', ':rostest', ':rosgraph', ':cmake_modules', ':rospack', ':rosbuild', ':roscpp', ':gencpp', ':rosparam', ':roscpp_serialization', ':roscpp_traits', ':rosout', ':rosclean', ':roslaunch', ':message_generation', ':roslang', ':genlisp', ':genpy', ':rostime', ':message_runtime', ':std_msgs', ':rospy', ':rosunit', ':rosgraph_msgs', ':genmsg', ':xmlrpcpp', ':cpp_common', ':rosmaster', ],
)
