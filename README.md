# ws-robotics-explorer

This repository for WRX (short for WhiteSponge Robotics Explorer) consists of the following:
- `wrx-motor-driver`: the Rust driver to read/broadcast encoder values from the motors and to drive/move them
- `serial-motor-rs`: the ROS2 node written in Rust that uses the encoder values from the wrx-motor-driver to calculate and publish the Odometry message
- `wrx-imu`: the Rust driver to get readings from the IMU sensor (BNO085 in this case) and broadcasts them
- `wrx-node-imu`: the ROS2 node written in Rust that processes the readings from the Rust driver above to calculate the angular velocity/orientation quaternion to publish in the IMU message
- `wrx-launch-controller`: the main launch package that describes the physical dimensions of the robot, LIDAR and what not. You might need to modify some of the values in the .xacro files to match your own physical robot. This is located in /colcon/src for easy usage with the colcon build tool in Ubuntu ROS2

* Note that the Rust drivers (mainly `wrx-motor-driver` and `wrx-imu`) are written with the Arduino Uno R3 in mind. If you're using a different microcontroller, you might need to refactor them)
