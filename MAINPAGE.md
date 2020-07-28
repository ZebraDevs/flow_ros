# Flow-ROS

This library is meant for synchronizing multiple series of ROS messages, collected by `ros::Subscriber` objects. This is typically desirable in ROS nodes where multiple inputs are required to compute something, and the computed result is accurate only if the inputs have consistent relative sequencing. This library uses [Flow](https://github.com/fetchrobotics/flow) to fulfill these requirements in a customizable way.

Secondarily, this library provides in-process subscriber/publisher mechanisms which do not require a running ROS core and are swappable with their ROS-enabled counterparts. These are especially useful for testing message passing subsystems.
