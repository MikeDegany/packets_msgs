# Networking in Collaborative Multi-Robot Systems for Enhanced Mapping and Navigation

## Table of Contents
1. [Introduction](#introduction)
2. [System Overview](#system-overview)
3. [Hardware Setup](#hardware-setup)
4. [Software Architecture](#software-architecture)
5. [Networking](#networking)
6. [Mapping](#mapping)
7. [Navigation](#navigation)
8. [Experiments and Results](#experiments-and-results)
9. [Conclusion](#conclusion)

## Introduction

This project is to implement Mapping and Navigation with multiple robots, meeting the growing need for efficient coordination in real-world tasks. It uses ROS2 and DDS for communication and data sharing between robots.

## System Overview

The system consists of multiple mobile robots interconnected via a Wi-Fi network, with a central PC handling the mapping and navigation tasks. The robots employ a Graph-based SLAM technique, integrating localized laser scans to construct and optimize a pose graph for a collaborative map.

![System Overview](path_to_system_overview_image.png)

## Hardware Setup

- Multiple TurtleBot4 Standard robots
- Fast and Powerful WiFi Router
- Central computer for data processing
Each robot is equipped with:
- iRobot Create3 mobile base
- Raspberry Pi 4B
- 2D LIDAR sensors
-  Wi-Fi card (iRobot Create3 only supports 2.4 GHz)

## Software Architecture

The software stack includes:
1. ROS2 for robot control and communication
2. DDS (Simple Discovery protocol) for efficient data distribution
3. DDS Domain ID bridge
4. SLAM-Toolbox for mapping
5. ROS2 navigation stack for autonomous navigation

## Networking

- Wi-Fi network connecting all robots and the central computer
- Use of domain bridge to optimize communication
- Implementation of namespacing and different Domain IDs for robot identification

### Network Performance Metrics
- Latency
- Packet loss
- Jitter

Refer to the included graphs for detailed performance analysis at various frequencies.

## Mapping

The system uses a Graph-based SLAM approach:
1. Robots equipped with 2D laser sensors
2. Localized laser scans
3. Pose graph creation
4. Map optimization

![Mapping Hierarchy](path_to_mapping_hierarchy_image.png)

Multiple robots share pose graphs to create a collaborative map.

## Navigation

The ROS NAV2 navigation stack is integrated for autonomous navigation:
- Path planning
- Obstacle avoidance
- Integration with SLAM systems

The navigation stack allows robots to navigate through dynamic environments and adapt to real-time changes.

## Experiments and Results

To set up multiple robots, start by configuring a single robot first. This manual provides the best guide to help you through the process: https://turtlebot.github.io/turtlebot4-user-manual/

Portions of the following documentation have been adapted from the original Turtlebot4 manual. In this tutorial we will be using ROS2 Humble.


After you set up a single robot and learn how to do mapping and navigation with it. Now it's time to set up multiple robots:

By default, each TurtleBot 4 will use the same [topic, action, and service names](../software/turtlebot4_common.md#ros-2-interfaces) for communication. If we connect two default TurtleBot 4's to the same network, the topics from both robots will communicate with each other and cause unwanted behaviours.

There are two main methods for running multiple TurtleBot 4's on a single network. _ROS_DOMAIN_ID_ and _NAMESPACING_. In this project we take both approaches together. 


## ROS_DOMAIN_ID

The **ROS_DOMAIN_ID** is an environment variable used in ROS 2 to change the ports that ROS 2 processes use for communication. This effectively allows us to separate different ROS 2 processes from communicating with each other on the same network. For a detailed explanation, click [here](https://docs.ros.org/en/galactic/Concepts/About-Domain-ID.html).

We can leverage this by assigning a unique **ROS_DOMAIN_ID** to each of our robots. This will ensure that communication between robots does not occur. The **ROS_DOMAIN_ID** can be set to any value between 0 and 101, inclusively.

### Setting the environment variable

There are four locations in which the **ROS_DOMAIN_ID** environment variable must be set:
 - Create® 3
 - RPi4 Terminal
 - RPi4 Robot Upstart Job
 - User PC


SSH into your TurtleBot 4 and run the turtlebot4 setup tool:

```bash
turtlebot4-setup
```

Navigate to 'Bash Setup' in the 'ROS Setup' menu, then change your `ROS_DOMAIN_ID`. Save the settings, then apply settings in the main menu.

<figure class="aligncenter">
    <img src="media/domain_id.gif" alt="ROS_DOMAIN_ID" style="width: 100%"/>
    <figcaption>Setting the ROS_DOMAIN_ID</figcaption>
</figure>

This will apply the new `ROS_DOMAIN_ID` to the Create® 3, RPi4 Terminal, and RPi4 Robot Upstart job.

On the user PC, set the `ROS_DOMAIN_ID` in your *setup.bash* file and source it. See [Installing ROS 2](../setup/basic.md#installing-ros-2) for more details.


Once the Create® 3 application has restarted, try calling `ros2 topic list` on both your PC and the RPi4 to ensure that all topics are visible.


## Namespacing

Namespacing is a method of adding a prefix to topic names to group certain topics, or to make them unique. 
By namespacing all topics on a TurtleBot 4, we can have a full set of unique topics for that robot. This allows
us to run multiple robots on the same `ROS_DOMAIN_ID`.

It is common to use the name of the robot as the namespace. For example, 
if we have a robot named `robot1`, the namespaced topics would look like:

```bash
ubuntu@ubuntu:~$ ros2 topic list
/parameter_events
/TB1/battery_state
/TB1/cmd_audio
/TB1/cmd_lightring
/TB1/cmd_vel
/TB1/diagnostics
/TB1/dock_status
/TB1/function_calls
/TB1/hazard_detection
/TB1/imu
/TB1/interface_buttons
/TB1/ip
/TB1/ir_intensity
/TB1/ir_opcode
/TB1/joint_states
/TB1/joy
/TB1/joy/set_feedback
/TB1/kidnap_status
/TB1/mobility_monitor/transition_event
/TB1/mouse
/TB1/odom
/TB1/robot_description
/TB1/robot_state/transition_event
/TB1/scan
/TB1/slip_status
/TB1/static_transform/transition_event
/TB1/stop_status
/TB1/tf
/TB1/tf_static
/TB1/wheel_status
/TB1/wheel_ticks
/TB1/wheel_vels
/rosout
```

Namespacing is not supported in Galactic.


To set the robot namespace, SSH into your TurtleBot 4 and run the turtlebot4 setup tool:

```bash
turtlebot4-setup
```

Navigate to 'Bash Setup' in the 'ROS Setup' menu, then change the `ROBOT_NAMESPACE` setting. 
Save the settings, then apply settings in the main menu.

<figure class="aligncenter">
    <img src="media/namespace.gif" alt="namespace" style="width: 100%"/>
    <figcaption>Setting the robot namespace</figcaption>
</figure>

This will apply the new namespace to the Create® 3, RPi4 Terminal, and RPi4 Robot Upstart job.


Assigning separate ROS_DOMAIN_IDs to the devices in the network will have several benefits, but the PC cannot see the robots in the networks, means the PC cannot subscribe to the topics published by robots. Therefore, we need to use a bridge to transform data from a DOMAIN_ID to another. 
This would be done using [domain_bridge](https://github.com/ros2/domain_bridge), a ROS 2 domain bridge. Bridges ROS communication between different ROS domain IDs.






### Viewing the Robot in RViz

On the user PC, `turtlebot4_desktop` launch files can use a `namespace` argument to view a specific robot:

```bash
ros2 launch turtlebot4_viz view_model.launch.py namespace:=/TB1
```

### Launching Robots in Simulation

The first robot can be launched normally, with the addition of a `namespace`. All other parameters are still available as shown below:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py namespace:=/TB1 nav2:=true slam:=false localization:=true rviz:=true
```

Any additional robots must be launched using the `turtlebot4_spawn` launch file, a unique `namespace` and a unique spawn location:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py namespace:=/TB2 x:=0.0 y:=1.0 nav2:=true slam:=false localization:=true rviz:=true
```

### Launching Navigation

The SLAM, Localization and Nav2 launch files all support namespacing and can be launched as follows:

```bash
ros2 launch turtlebot4_navigation slam.launch.py namespace:=/TB1
ros2 launch turtlebot4_navigation localization.launch.py map:=office.yaml namespace:=/TB1
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/TB1
```

Replace `TB1` with the desired robot namespace. 





Experiments were conducted to evaluate the performance and efficiency of the multi-robot mapping and navigation system in an indoor environment.

### Setup
- Multiple TurtleBot3 robots
- iRobot Create3 mobile base
- Raspberry Pi 4 running ROS2
- 2D LIDAR sensors
- Central computer for coordination and data aggregation

### Results
- Successful collaborative mapping of indoor environments
- Improved latency and reduced packet loss with domain bridge implementation
- Efficient autonomous navigation using the generated map

Refer to the included figures for visual results of the mapping process and network performance graphs.

## Conclusion

This project successfully addressed the challenges of mapping and navigation with multiple robots through a comprehensive approach encompassing networking, mapping, and navigation. The system demonstrates the feasibility and potential applications of multi-robot systems in various real-world scenarios.
