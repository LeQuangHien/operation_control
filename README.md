# operation_control
This is operation package in Ground Control Station (GCS) which is one of 6 packages in the ros2 evaluation workingspace. This experiment aims to evaluate the Quality of Service (QoS) in ROS2.

![QoS_ROS2_2](https://user-images.githubusercontent.com/9337121/121939049-d635f100-cd4c-11eb-8839-25632bb8f8c3.png)


## List of packages
1. [px4_ros_com](https://github.com/LeQuangHien/px4_ros_com)
2. [px4_msgs](https://github.com/LeQuangHien/px4_msgs)
3. [bridge_msgs](https://github.com/LeQuangHien/bridge_msgs)
4. [operation_control](https://github.com/LeQuangHien/operation_control)
5. [image_tools](https://github.com/LeQuangHien/image_tools)
6. [launch_bringup](https://github.com/LeQuangHien/launch_bringup)

## How to build
- [Install ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html)
- [Install PX4](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html)
- [Install Fast DDS](https://docs.px4.io/master/en/dev_setup/fast-dds-installation.html)
- Building ros2 evaluation working space
  1. Create a workspace directory using:
  ```
  mkdir -p ~/ros2_evaluation/src
  ```
  2. Clone 6 packages above `px4_ros_com`, `px4_msgs`, `bridge_msgs`, `operation_control`, `image_tools`, `launch_bringup` to the /src directory (the master branch is cloned by default):
  ```
  cd ~/ros2_evaluation/src
  git clone https://github.com/LeQuangHien/px4_ros_com
  git clone https://github.com/LeQuangHien/px4_msgs
  git clone https://github.com/LeQuangHien/bridge_msgs
  git clone https://github.com/LeQuangHien/operation_control
  git clone https://github.com/LeQuangHien/image_tools
  git clone https://github.com/LeQuangHien/launch_bringup
  ```
  3. Build the `ros2_evaluation` workingspace, remember source ros2 ws 
  ```
  cd ..
  . ~/ros2_foxy/install/local_setup.bash
  colcon build --symlink-install --event-handlers console_direct+
  ```
  4. [Sanity Check the Installation](https://docs.px4.io/master/en/ros/ros2_comm.html#sanity-check-the-installation)
   
## How to run
Two PCs are needed to run this experiment. One for the offboard computer and one for Ground Control Station(GCS). Above section "How to build" should be completed in both PCs. Both PCs should be in the same wireless network.

1. In the offboard computer
  - Open a new terminal in the root of the PX4 Autopilot project, and then start a PX4 Gazebo simulation using:
  ```
  make px4_sitl_rtps gazebo
  ```
  - On a new terminal, source the ROS 2 workspace and then start the micrortps_agent daemon with UDP as the transport protocol:
  ```
  source ~/ros2_evaluation/install/setup.bash
  micrortps_agent -t UDP
  ```
  - On the original terminal (System console) start the micrortps_client daemon with UDP:
  ```
  pxh> micrortps_client start -t UDP
  ```
  - Open a new terminal and start "offboard_control" node using:
  ```
  source ~/ros2_evaluation/install/setup.bash
  ros2 run px4_ros_com offboard_control  # default QoS policies
  ```
  or
  ```
  source ~/ros2_evaluation/install/setup.bash
  ros2 run px4_ros_com offboard_control --ros-args -p reliability:=best_effort  # best-effort QoS policies
  ```
  - Open a new terminal and start publishing images and sensor data using launch file:
  ```
  source ~/ros2_evaluation/install/setup.bash
  ros2 launch launch_bringup drone_default_qos.launch.py  # default QoS policies
  ```
  or
  ```
  source ~/ros2_evaluation/install/setup.bash
  ros2 launch launch_bringup drone_best_effort_qos.launch.py  # best-effort QoS policies
  ```
2. In the GCS
  - Open a new terminal and start "setpoint_advertiser" node using:
  ```
  source ~/ros2_evaluation/install/setup.bash
  ros2 run operation_control setpoint_advertiser # default QoS policies
  ```
  or
  ```
  source ~/ros2_evaluation/install/setup.bash
  ros2 run operation_control setpoint_advertiser --ros-args -p reliability:=best_effort  # best-effort QoS policies
  ```
  - Open a new terminal to receive images and sensor data from drone using launch file:
  ```
  source ~/ros2_evaluation/install/setup.bash
  ros2 launch launch_bringup gcs_default_qos.launch.py  # default QoS policies
  ```
  or
  ```
  source ~/ros2_evaluation/install/setup.bash
  ros2 launch launch_bringup gcs_best_effort_qos.launch.py  # best-effort QoS policies
  ```
3. In the offboard computer
  - Open a new terminal to observe published statistic data for "Setpoint":
  ```
  ros2 topic echo /statistics_setpoint
  ```
  Data type in the statistics
  ![data_type](https://user-images.githubusercontent.com/9337121/124759851-691f1100-df30-11eb-9b92-b7f669b6e050.png)

4. In the GCS
  - Open a new terminal to observe published statistic data for "Sensor":
  ```
  ros2 topic echo /statistics_sensor
  ```
5. Run `ifconfig` to get the interface for communicating between devices in the local network, here is "wlp2s0". Then simulate 10% packet loss using "tc" in both the offboard computer and the GCS
  ```
  sudo tc qdisc add dev wlp2s0 root netem loss 10%  # Remember to delete this by using: `sudo tc qdisc delete dev wlp2s0 root netem loss 10%`
  ```
6. Observe the outputs in "offboard_control" terminal and the statistic's terminals. Compare the results with and without packet loss, with and without "best_effort" QoS policy.


## References
- [ROS 2 User Guide (PX4-ROS 2 Bridge)](https://docs.px4.io/master/en/ros/ros2_comm.html)
- [ROS 2 Offboard Control Example](https://docs.px4.io/master/en/ros/ros2_offboard_control.html)
- [Use quality-of-service settings to handle lossy networks](https://docs.ros.org/en/foxy/Tutorials/Quality-of-Service.html)
- [ROS 2 Topic Statistics Tutorial (C++)](https://docs.ros.org/en/foxy/Tutorials/Topics/Topic-Statistics-Tutorial.html)
