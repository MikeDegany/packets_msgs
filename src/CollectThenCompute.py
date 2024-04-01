#!/usr/bin/env python3

#Maintainer: Mike Degany (mike.degany@gmail.com)

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
import signal
# import numpy as np
import seaborn as sns
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy



#Store messages in a list to be used for calculating latency later
packets = []
#Create a Node class (listener) that will subscribe to the packets topic and store the latency of each message
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('listener')
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(
            Packet, # Message type
            'packets',  # Topic name
            self.listener_callback, # Callback function
            qos_profile  # Pass the QoS profile
            # 1
            )
        self.subscription  # prevent unused variable warning
        self.received_message = False


    def listener_callback(self, msg):
        msg.rec_stamp = self.get_clock().now().to_msg()
        packets.append(msg)
        self.received_message = True

    def check_for_timeout(self):
        if not self.received_message:
            self.get_logger().info("No messages received. Exiting...")
            rclpy.shutdown()



def calculate_timediffs(packets):
    latency_lists = {}
    for msg in packets:
        if msg.freq not in latency_lists:
            latency_lists[msg.freq] = []  # Initialize list if category is encountered for the first time
        time_diff_sec = msg.rec_stamp.sec - msg.stamp.sec
        time_diff_nsec = msg.rec_stamp.nanosec - msg.stamp.nanosec
        time_diff_nsec *= 10**-9
        time_diff = time_diff_sec + time_diff_nsec
        # latency_lists[msg.freq].append(time_diff)
        if time_diff > 0 and time_diff < 0.01:
            latency_lists[msg.freq].append(time_diff)
        else:
            print(f"oulier detected: {msg.freq}  {time_diff}")
    return latency_lists

def plot_jitter_profiles(output_folder, dict_of_lists):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    for category, times in dict_of_lists.items():
        plt.figure()
        plt.plot(times)
        plt.xlabel('Index')
        plt.ylabel('Time (sec)')
        plt.title(f'Jitter plot - Category {category}')
        plt.savefig(os.path.join(output_folder, f'jitter_plot_category_{category}.png'))
        plt.close()

def calculate_statistics(latency_lists):
    statistics = {}
    for frequency, delays in latency_lists.items():
        mean = sum(delays) / len(delays)
        variance = sum((x - mean) ** 2 for x in delays) / len(delays)
        statistics[frequency] = {'mean': mean, 'variance': variance}
    for frequency, stats in statistics.items():
        print(f"frequency: {frequency}")PLR
        print(f"Mean: {stats['mean']}")
        print(f"Variance: {stats['variance']}")
        print("-------------------------------")
    return statistics


def generate_histogram(output_folder, latency_lists):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    for Frequency, stats in latency_lists.items():
        ax = sns.histplot(latency_lists[Frequency], bins=50, alpha=0.5, label=Frequency)
        mids = [rect.get_x() + rect.get_width() / 2 for rect in ax.patches]
        plt.xlabel('Latency (sec)')PLR
        plt.ylabel('occurance')
        plt.title(f'Jitter Histogram - Frequency: {Frequency} Hz')
        plt.savefig(os.path.join(output_folder, f'jitter_histogram_Frequency_{Frequency}.png'))
        plt.close()


def plot_boxplot(output_folder, latency_lists):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    # Prepare data for boxplot
    data = [times for times in latency_lists.values()]
    labels = list(latency_lists.keys())

    # Plot boxplot
    plt.figure()
    plt.boxplot(data, labels=labels)
    plt.xlabel('Category')
    plt.ylabel('Time (sec)')
    plt.title('Boxplot of Latency Categories')
    plt.savefig(os.path.join(output_folder, f'Jitter_Boxplot.png'))
    plt.close()

def calculate_packet_loss(latency_lists):
    packet_loss_rates = {}
    for category, times in latency_lists.items():
        num_packets_received = len(times)
        packet_loss = 1000 - num_packets_received
        packet_loss_rate = packet_loss / 1000
        packet_loss_rates[category] = packet_loss_rate
    return packet_loss_rates

def plot_packet_loss(output_folder, packet_loss_rates):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    categories = list(packet_loss_rates.keys())
    loss_rates = list(packet_loss_rates.values())

    plt.figure()
    plt.plot(categories, loss_rates, marker='o')
    plt.xlabel('Frequency')
    plt.ylabel('Packet Loss Rate')
    plt.title('Packet Loss Rate vs Frequency')
    plt.grid(True)
    plt.savefig(os.path.join(output_folder, f'jitter_histogram_Frequency_{categories}.png'))
    plt.close()



def signal_handler(sig, frame):
    global packets
    latency_lists = calculate_timediffs(packets)
    print("Ctrl+C pressed. Exiting...")
    # print("Latency lists:", latency_lists)
    statistics = calculate_statistics(latency_lists)
    generate_histogram('/home/UNT/md0708/plots/hists', latency_lists)
    plot_boxplot('/home/UNT/md0708/plots/boxplot', latency_lists)  # Call plot_boxplot function here
    plot_jitter_profiles('/home/UNT/md0708/plots/plots', latency_lists)
    packet_loss_rates = calculate_packet_loss(latency_lists)
    print("Packet Loss Rates:", packet_loss_rates)
    plot_packet_loss('/home/UNT/md0708/plots/PLR',packet_loss_rates)

    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    global latency_lists

    # Register the signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    minimal_subscriber = MinimalSubscriber()
    minimal_subscriber.create_timer(2, minimal_subscriber.check_for_timeout)

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
