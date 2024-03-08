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

from std_msgs.msg import String

import os

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(Packet, 'packets', 1)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

#     def timer_callback(self):
#         ROS_DOMAIN_ID = os.environ.get('ROS_DOMAIN_ID')
#         msg = String()
# #        msg.data = 'Hello Mike: %d' % self.i
#         current_time = self.get_clock().now()
#         print(type(current_time))
#         # print(type(current_time))
#         msg.data = f'{current_time.nanoseconds}: TB{ROS_DOMAIN_ID}: {self.i}'
#         self.publisher_.publish(msg)
#         self.get_logger().info(' "%s"' % msg.data)
#         self.i += 1


    def timer_callback(self):
        ROS_DOMAIN_ID = os.environ.get('ROS_DOMAIN_ID')
        msg = Packet()
        msg.num = self.i
        # current_time = self.get_clock().now()
        # print(type(current_time))
        # print(type(current_time))
        # msg.sec = current_time.nanoseconds
        self.publisher_.publish(msg)
        self.get_logger().info(' "%s"' % msg.num)
        self.i += 1




def main(args=None):
    rclpy.init(args=args)

    talker = MinimalPublisher()

    while rclpy.ok() and talker.i < 50000:
        rclpy.spin_once(talker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


