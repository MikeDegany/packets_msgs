#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from packets_msgs.msg import Packet
from std_msgs.msg import Header
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('talker')
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.publisher_ = self.create_publisher(Packet, 'packets', qos_profile)
        # self.timer_period = timer_period  # seconds
        # print(f'time period is: "{self.timer_period}"')
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.msg = Packet()
        self.ROS_DOMAIN_ID = int(os.environ.get('ROS_DOMAIN_ID', 0))

        # Wait for the publisher to be fully initialized
        while not self.publisher_.get_subscription_count():
            self.get_logger().info('Waiting for subscriber...')
            rclpy.spin_once(self, timeout_sec=0.1)

    def update_timer_period(self, timer_period):
        if hasattr(self, 'timer'):
            self.destroy_timer(self.timer)
        self.timer_period = timer_period
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        self.msg.stamp = self.get_clock().now().to_msg()
        self.msg.domain_id = self.ROS_DOMAIN_ID
        self.msg.packet_id = self.i
        self.msg.freq = int(1 / self.timer_period)
        self.publisher_.publish(self.msg)
        # self.get_logger().info(' "%s"' % msg.stamp)
        # self.get_logger().info(f'time period is: "{self.timer_period}" and self.i is: "{self.i}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = MinimalPublisher()


    # # Wait for the publisher to be fully initialized
    # while not talker.publisher_.get_subscription_count():
    #     rclpy.spin_once(talker, timeout_sec=0.1)


    timer_periods = [0.1, 0.02, 0.01, 0.005, 0.002, 0.001, 0.0005, 0.00025, 0.000166, 0.000125, 0.0001]
    for period in timer_periods:
        # Update the timer period dynamically
        talker.update_timer_period(period)
        for _ in range(1000):
            if not rclpy.ok():
                break
            rclpy.spin_once(talker)

    talker.destroy_node()
    rclpy.shutdown()





    # for i in [0.01, 0.005, 0.002, 0.001, 0.0005, 0.00025, 0.000166, 0.000125, 0.0001]:
    #     # talker = MinimalPublisher(i)
    #     for _ in range(1000):
    #         if not rclpy.ok():
    #             break
    #         rclpy.spin_once(talker)
    #     talker.destroy_node()

    # rclpy.shutdown()


if __name__ == '__main__':
    main()
