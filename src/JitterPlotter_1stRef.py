#!/usr/bin/env python3

#Maintainer: Mike Degany (mike.degany@gmail.com)

# Copyright 2024 Mike Degany.
# Licensed under the Attribution-Only License, Version 1.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# https://example.com/license/attribution-only-v1.0
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
import seaborn as sns
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from datetime import datetime
import json



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
        msg.rec_stamp = self.get_clock().now().to_msg() # Record the time the message was received
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
            TOC = msg.stamp.sec + (msg.stamp.nanosec * 10**-9)
            i_TOC = msg.packet_id
        # time_diff_sec = msg.rec_stamp.sec - msg.stamp.sec
        # time_diff_nsec = msg.rec_stamp.nanosec - msg.stamp.nanosec
        # time_diff_nsec *= 10**-9
        # time_diff = time_diff_sec + time_diff_nsec
        try:
            time_diff = (msg.rec_stamp.sec + (msg.rec_stamp.nanosec * 10**-9)) - ( TOC + (msg.packet_id - i_TOC) * 1/msg.freq)
        except ZeroDivisionError:
            # Handle division by zero error here
            print("Error: Division by zero!")
            print(f"msg.freq: {msg.freq}")
        latency_lists[msg.freq].append(time_diff)
        # if time_diff > 0 and time_diff < 0.01:
        #     latency_lists[msg.freq].append(time_diff)
        # else:
        #     print(f"oulier detected: {msg.freq}  {time_diff}")
    return latency_lists

def plot_jitter_profiles(output_folder, dict_of_lists):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    for category, times in dict_of_lists.items():
        plt.figure()
        plt.scatter(range(len(times)), times, marker='o', color='b', alpha=0.5)
        plt.xlabel('Packet Index')
        plt.ylabel('Time Delay (sec)')
        plt.title(f'Jitter plot - Category {category}')

        # Calculate average time delay
        avg_delay = np.mean(times)

        # Plot average line
        plt.axhline(y=avg_delay, color='r', linestyle='--', label=f'Average: {avg_delay:.6f} sec')
        plt.legend()

        plt.savefig(os.path.join(output_folder, f'jitter_plot_category_{category}.pdf'), format='pdf')
        plt.close()

def calculate_statistics(latency_lists):
    statistics = {}
    for frequency, delays in latency_lists.items():
        mean = sum(delays) / len(delays)
        variance = sum((x - mean) ** 2 for x in delays) / len(delays)
        statistics[frequency] = {'mean': mean, 'variance': variance}

    for frequency, stats in statistics.items():
        print(f"frequency: {frequency}")
        print(f"Mean: {stats['mean']}")
        print(f"Variance: {stats['variance']}")
        print("-------------------------------")
    return statistics



def generate_histogram(output_folder, latency_lists):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    for Frequency, data in latency_lists.items():
        plt.figure(figsize=(10, 6))  # Adjust the figure size
        sns.set_theme(style="whitegrid")   # Set seaborn style

        # Plot histogram
        sns.histplot(data, bins=50, alpha=0.5, color='skyblue')

        # Add mean and standard deviation lines
        mean = np.mean(data)
        std_dev = np.std(data)
        plt.axvline(mean, color='r', linestyle='--', linewidth=2, label=f'Mean: {mean:.6f}')
        plt.axvline(mean - std_dev, color='g', linestyle='--', linewidth=2, label=f'Std Dev: {std_dev:.6f}')
        plt.axvline(mean + std_dev, color='g', linestyle='--', linewidth=2)

        # Add labels and title
        plt.xlabel('Latency (sec)', fontsize=14)
        plt.ylabel('Frequency', fontsize=14)
        plt.title(f'Jitter Histogram - Frequency: {Frequency} Hz', fontsize=16)

        # Add legend
        plt.legend()

        # Save the plot
        plt.savefig(os.path.join(output_folder, f'jitter_histogram_Frequency_{Frequency}.pdf'), format='pdf')
        plt.close()

def plot_boxplot(output_folder, latency_lists):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Prepare data for boxplot
    data = [times for times in latency_lists.values()]
    labels = list(latency_lists.keys())

    # Set seaborn style
    sns.set(style="whitegrid")

    # Plot boxplot
    plt.figure(figsize=(10, 6))  # Adjust the figure size
    sns.boxplot(data=data, palette="Set3")

    # Add labels and title
    plt.xlabel('Category', fontsize=14)
    plt.ylabel('Latency (sec)', fontsize=14)
    plt.title('Boxplot of Latency Categories', fontsize=16)

    # Rotate x-axis labels for better readability
    plt.xticks(rotation=45)

    # Save the plot
    plt.savefig(os.path.join(output_folder, 'Jitter_Boxplot.pdf'), format='pdf')
    plt.close()

def calculate_packet_loss(latency_lists):
    packet_loss_rates = {}
    print("Packet Loss Rates:")
    for category, times in latency_lists.items():
        num_packets_received = len(times)
        if category <= 100:
            packet_loss = 10*category - num_packets_received
            packet_loss_rate = packet_loss / 10*category
        else:
            packet_loss = 1000 - num_packets_received
            packet_loss_rate = packet_loss / 1000
        packet_loss_rates[category] = packet_loss_rate * 100
        print(f'Frequency:{category} | {packet_loss}')
    return packet_loss_rates

def plot_packet_loss(output_folder, packet_loss_rates):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    categories = packet_loss_rates.keys()
    loss_rates = packet_loss_rates.values()

    # Plotting the data
    plt.figure(figsize=(10, 6))
    plt.plot(categories, loss_rates, marker='o', linestyle='-')
    plt.title('Packet Loss Rates')
    plt.xlabel('Time (ms)')
    plt.ylabel('Loss Rate (%)')
    plt.xscale('log')  # Logarithmic scale for x-axis
    plt.grid(True, which="both", ls="--", alpha=0.5)
    # Annotate each point with its percentage value
    for category, loss_rate in zip(categories, loss_rates):
        plt.text(category, loss_rate, f'{loss_rate:.2f}%\n{category}', ha='right', va='bottom')
    plt.tight_layout()
    plt.savefig(os.path.join(output_folder, 'packet_loss_vs_frequency.pdf'), format='pdf')  # Save figure
    plt.close()


def save_data(output_folder, filename, latency_lists):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    try:
        with open(os.path.join(output_folder, f'{filename}.json'), 'w') as f:
            json.dump(latency_lists, f)
    except Exception as e:
        print(f"Error saving data: {e}")

def load_data(output_folder):
    with open(os.path.join(output_folder, 'latency_lists.json'), 'r') as f:
        latency_lists = json.load(f)
    return latency_lists

def signal_handler(sig, frame):
    global packets
    latency_lists = calculate_timediffs(packets)
    print("Ctrl+C pressed. Exiting...")
    # print("Latency lists:", latency_lists)

    ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")  # begining timestamp
    run_path = os.path.join("/home/UNT/md0708/Experiments/", ts + "_CollectThenCompute")

    boxplot_path = os.path.join(run_path, "boxplot")
    hists_path = os.path.join(run_path, "hists")
    plots_path = os.path.join(run_path, "plots")
    PLR_path = os.path.join(run_path, "PLR")
    packets_path = os.path.join(run_path, "packets")
    Data_path = os.path.join(run_path, "Data")
    os.makedirs(boxplot_path)
    os.makedirs(hists_path)
    os.makedirs(plots_path)
    os.makedirs(PLR_path)
    os.makedirs(Data_path)
    statistics = calculate_statistics(latency_lists)
    generate_histogram(hists_path, latency_lists)
    plot_boxplot(boxplot_path, latency_lists)  # Call plot_boxplot function here
    plot_jitter_profiles(plots_path, latency_lists)
    packet_loss_rates = calculate_packet_loss(latency_lists)
    print("Packet Loss Rates:", packet_loss_rates)
    plot_packet_loss(PLR_path,packet_loss_rates)

    save_data(Data_path, 'latency_lists', latency_lists)
    save_data(Data_path, 'packets', str(packets))
    save_data(Data_path, 'statistics', statistics)
    save_data(Data_path, 'packet_loss_rates', packet_loss_rates)
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    global latency_lists

    # Register the signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    minimal_subscriber = MinimalSubscriber()
    minimal_subscriber.create_timer(11, minimal_subscriber.check_for_timeout)

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

