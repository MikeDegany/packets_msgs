#!/usr/bin/env python3


# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from packets_msgs.msg import Packet

from std_msgs.msg import Header

import os

class MinimalPublisher(Node):

    def __init__(self, timer_period=0.01):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(Packet, 'packets', 1)
        self.timer_period = timer_period  # seconds
        print(f'time period is: "{self.timer_period}"')
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        ROS_DOMAIN_ID = os.environ.get('ROS_DOMAIN_ID')
        msg = Packet()
        msg.stamp = self.get_clock().now().to_msg()
        msg.domain_id = int(ROS_DOMAIN_ID)
        msg.packet_id = self.i
        msg.freq = int(1/self.timer_period)
        self.publisher_.publish(msg)
        self.get_logger().info(' "%s"' % msg.stamp)
        self.get_logger().info(f'time period is: "{self.timer_period}" and self.i is: "{self.i}"')
        self.i += 1




def main(args=None):
    rclpy.init(args=args)

    for i in [0.0001, 0.001, 0.01, 0.1]:
        talker = MinimalPublisher(i)
        for _ in range(50):
            if not rclpy.ok():
                break
            rclpy.spin_once(talker)
        talker.destroy_node()



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
   
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


