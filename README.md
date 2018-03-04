# BallCollectorProject
This repository contains code that can be used to control VEX EDR robots by
using a serial link between the VEX Cortex-based Microcontroller and another
machine with available serial pins, such as a Raspberry Pi.

It currently supports:
 * Reliable serial-based communication between the VEX Cortex and the ROS controller machine
 * Differential drive base control using the `/cmd_vel` topic
 * Odometry using VEX sensors (such as IMEs)

Most of the interesting code is in the `cortex_driver` package (in `src/cortex_driver`).
This package contains three node types:
 * `robot_driver`, which handles communications with the Cortex, as well as computing odometry
 * `base_controller`, which reads `geometry_msgs/Twist` messages from the
   `/cmd_vel` topic and computes left/right wheel velocities to send to `robot_driver`.
 * `drive_console`, which implements a very simple interface for driving the robot manually.
