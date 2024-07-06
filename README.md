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
