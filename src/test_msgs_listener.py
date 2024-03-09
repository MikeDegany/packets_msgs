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
from packets_msgs.msg import Packet
import matplotlib.pyplot as plt
from std_msgs.msg import String
import os


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            Packet,
            'packets',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.latency_lists = {}
        self.received_message = False

    def listener_callback(self, msg):
        current_time = self.get_clock().now().to_msg()
        time_diff_sec = current_time.sec - msg.stamp.sec
        time_diff_nsec = current_time.nanosec - msg.stamp.nanosec
        time_diff_nsec *= 10**-9
        # self.latencies.append(time_diff_nsec)
        freq = msg.freq
        if freq not in self.latency_lists:
            self.latency_lists[freq] = []  # Initialize list if category is encountered for the first time
        self.latency_lists[freq].append(time_diff_nsec)
        self.received_message = True
        # print(self.latency_lists)

        #self.get_logger().info(f'sec and nsec are: "{time_diff_sec}.{time_diff_nsec}"')
        #print(type(current_time))
        #print(type(msg.stamp))
        self.get_logger().info(f'Message #{msg.packet_id} from {msg.domain_id} with frequency {msg.freq} Hz')
        self.get_logger().info(f'Time difference is: "{time_diff_sec}.{time_diff_nsec}"')
        # if len(self.latencies) == 1000:
        #     self.plot_jitter_profile()
                # self.destroy_node()
                # rclpy.shutdown()
        self.plot_jitter_profiles('/home/UNT/md0708/plots')



    def plot_jitter_profiles(self, output_folder):
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        for category, times in self.latency_lists.items():
            plt.figure()
            plt.plot(times)
            plt.xlabel('Index')
            plt.ylabel('Time (nanoseconds)')
            plt.title(f'Jitter Profile - Category {category}')
            plt.savefig(os.path.join(output_folder, f'jitter_profile_category_{category}.png'))
            plt.close()


    # def wait_for_messages(self, timeout_sec):
    #     future = self.create_future()
    #     self.subscription = self.create_subscription(
    #         String,
    #         'topic',
    #         self.listener_callback,
    #         1)
    #     rclpy.spin_until_future_complete(self, future, timeout_sec)
    #     if not self.received_message:
    #         print("No messages received within the timeout period.")

    # def plot_jitter_profile(self):
    #     plt.plot(self.latencies)
    #     plt.xlabel('Message Index')
    #     plt.ylabel('Latency (s)')
    #     plt.title('Jitter Profile')
    #     plt.grid(True)
    #     plt.show()


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    # future = minimal_subscriber.wait_for_message(timeout_sec=10.0)  # Wait for a message for up to 10 seconds
    # print("Shutting down")

    # try:
    #     minimal_subscriber.wait_for_messages(10.0)  # Wait for 2 seconds for messages
    # finally:
    #     minimal_subscriber.destroy_node()
    #     rclpy.shutdown()
    rclpy.spin(minimal_subscriber)
    # minimal_subscriber.plot_jitter_profiles('/home/UNT/md0708/plots')
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
