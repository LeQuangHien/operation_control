# operation_control
This is operation package in Ground Control Station (GCS) which is one of 6 packages in the ros2 evaluation workingspace. This experiment aims to evaluate the Quality of Service (QoS) in ROS2.

![QoS_ROS2_2](https://user-images.githubusercontent.com/9337121/121939049-d635f100-cd4c-11eb-8839-25632bb8f8c3.png)


## List of packages
1. [px4_ros_com](https://github.com/LeQuangHien/px4_ros_com): is forked from [PX4-ROS2 bridge](https://github.com/PX4/px4_ros_com) and then add/edit to adapt the experiments.
2. [px4_msgs](https://github.com/LeQuangHien/px4_msgs): is forked from [ROS2 message definitions of the PX4 Pro ecosystem](https://github.com/PX4/px4_msgs).
3. [bridge_msgs](https://github.com/LeQuangHien/bridge_msgs): includes bridge messages for communication between the companion computer and the Ground Control Station.
4. [operation_control](https://github.com/LeQuangHien/operation_control): is the main package for GCS.
5. [image_tools](https://github.com/LeQuangHien/image_tools): is forked from ROS2 foxy source [code](https://github.com/ros2/ros2/releases/tag/release-foxy-20201211) and then edit to adapt the experiments.
6. [launch_bringup](https://github.com/LeQuangHien/launch_bringup): includes launch files in order to run required nodes in the companion computer and the Ground Control Station.

## How to build
- [Install ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html)
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
Two PCs are needed to run this experiment. One for the companion computer and one for Ground Control Station(GCS). Above section "How to build" should be completed in both PCs. Both PCs should be in the same wireless network.

1. In the companion computer
  
  ```
  # Open a new terminal in the root of the PX4 Autopilot project, and then start a PX4 Gazebo simulation using:
  make px4_sitl_rtps gazebo
  
  # On a new terminal, source the ROS 2 workspace and then start the micrortps_agent daemon with UDP as the transport protocol:
  source ~/ros2_evaluation/install/setup.bash
  micrortps_agent -t UDP
  
  # On the original terminal (System console of PX4 Gazebo) start the micrortps_client daemon with UDP:
  micrortps_client start -t UDP
  
  # Open a new terminal to launch drone nodes including offboard_control node and two nodes publishing images and sensor data (pick one of two below):
   source ~/ros2_evaluation/install/setup.bash
   
  # The failsafe mode of the drone simulator is usually enabled in the beginning, so if you see the drone has not taken off, then Ctrl+C to end and then run the launch file again. Do it untill the drone has taken off. In my case, the drone will take off in the second time
  
  # for default QoS polices
  ros2 launch launch_bringup drone_default_qos.launch.py
  # Or for best-effort QoS policies
  ros2 launch launch_bringup drone_best_effort_qos.launch.py
  ```
  
2. In the GCS
 
  ``` 
  # Open a new terminal and start setpoint_advertiser node and two nodes receiving images and sensor data from drone using launch file (pick one of two below):
  source ~/ros2_evaluation/install/setup.bash
  
  # for default QoS polices
  ros2 launch launch_bringup gcs_default_qos.launch.py
  # Or for best-effort QoS policies
  ros2 launch launch_bringup drone_best_effort_qos.launch.py
  ```
3. In the companion computer
 
  ```
  #  Open a new terminal to observe published statistic data for "Setpoint":
  ros2 topic echo /statistics_setpoint
  
  # For data type in the statistics, please see the lookup table below
  ```
  ![data_type](https://user-images.githubusercontent.com/9337121/124759851-691f1100-df30-11eb-9b92-b7f669b6e050.png)

4. In the GCS
  
  ```
  # Open a new terminal to observe published statistic data for "Sensor":
  ros2 topic echo /statistics_sensor
  # Open a new terminal to observe published statistic data for "Image":
  ros2 topic echo /statistics_image
  
  # For data type in the statistics, please see the lookup table below
  ```
  ![data_type](https://user-images.githubusercontent.com/9337121/124759851-691f1100-df30-11eb-9b92-b7f669b6e050.png)
  
5. Run `ifconfig` to get the interface for communicating between devices in the local network, here is "wlp2s0". Then simulate 10% packet loss using "tc" in both the offboard computer and the GCS
  ```
  sudo tc qdisc add dev wlp2s0 root netem loss 10%  
  
  # Remember to delete this by using: 
  sudo tc qdisc delete dev wlp2s0 root netem loss 10%
  ```
6. Observe the outputs in the statistic's terminals. Compare the results with and without packet loss, with and without "best_effort" QoS policy.


## References
- [ROS 2 User Guide (PX4-ROS 2 Bridge)](https://docs.px4.io/master/en/ros/ros2_comm.html)
- [ROS 2 Offboard Control Example](https://docs.px4.io/master/en/ros/ros2_offboard_control.html)
- [Use quality-of-service settings to handle lossy networks](https://docs.ros.org/en/foxy/Tutorials/Quality-of-Service.html)
- [ROS 2 Topic Statistics Tutorial (C++)](https://docs.ros.org/en/foxy/Tutorials/Topics/Topic-Statistics-Tutorial.html)
