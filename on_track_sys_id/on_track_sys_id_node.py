#!/usr/bin/env python3

import threading
import numpy as np
import os
import yaml
import csv

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from on_track_sys_id.helpers.train_model import nn_train
from datetime import datetime
from tqdm import tqdm


class OnTrackSysId(Node):
    def __init__(self):
        super().__init__('on_track_sys_id')

        self.declare_parameter('racecar_version', 'SIM')
        self.declare_parameter('save_LUT_name', 'NUCx_on_track_pacejka')
        self.declare_parameter('plot_model', True)
        self.declare_parameter('odom_topic', '/car_state/odom')
        self.declare_parameter('ackermann_cmd_topic', '/vesc/high_level/ackermann_cmd_mux/input/nav_1')

        self.racecar_version = self.get_parameter('racecar_version').value
        self.get_logger().info(f"Racecar_version: {self.racecar_version}")

        self.rate = 50
        self.package_path = get_package_share_directory('on_track_sys_id')

        self.load_parameters()
        self.setup_data_storage()

        self.save_LUT_name = self.get_parameter('save_LUT_name').value
        self.plot_model = self.get_parameter('plot_model').value

        odom_topic = self.get_parameter('odom_topic').value
        ackermann_topic = self.get_parameter('ackermann_cmd_topic').value

        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.create_subscription(AckermannDriveStamped, ackermann_topic, self.ackermann_cb, 10)

        self.collection_done = False
        self.pbar = tqdm(total=self.timesteps, desc='Collecting data', ascii=True)
        self.timer = self.create_timer(1.0 / self.rate, self.collect_data)

    def setup_data_storage(self):
        """
        Set up storage for collected data.

        Initializes data storage and related variables based on parameters loaded from 'nn_params'.
        """
        self.data_duration = self.nn_params['data_collection_duration']
        self.timesteps = self.data_duration * self.rate
        self.data = np.zeros((self.timesteps, 4))
        self.counter = 0
        self.current_state = np.zeros(4)

    def load_parameters(self):
        """
        This function loads parameters neural network parameters from 'params/nn_params.yaml' and stores them in self.nn_params.
        """
        yaml_file = os.path.join(self.package_path, 'params/nn_params.yaml')
        with open(yaml_file, 'r') as file:
            self.nn_params = yaml.safe_load(file)

    def export_data_as_csv(self):
        """
        Export collected data as a CSV file.

        Prompts the user to confirm exporting data. If confirmed, data is saved as a CSV file
        including velocity components (v_x, v_y), yaw rate (omega), and steering angle (delta).
        """
        user_input = input("\033[33m[WARN] Press 'Y' and then ENTER to export data as CSV, or press ENTER to continue without dumping: \033[0m")
        if user_input.lower() == 'y':
            data_dir = os.path.join(self.package_path, 'data', self.racecar_version)
            if not os.path.exists(data_dir):
                os.makedirs(data_dir)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_file = os.path.join(data_dir, f'{self.racecar_version}_sys_id_data_{timestamp}.csv')

            with open(csv_file, mode='w') as file:
                writer = csv.writer(file)
                writer.writerow(['v_x', 'v_y', 'omega', 'delta'])
                for row in self.data:
                    writer.writerow(row)
            self.get_logger().info(f"DATA HAS BEEN EXPORTED TO: {csv_file}")

    def odom_cb(self, msg):
        self.current_state[0] = msg.twist.twist.linear.x
        self.current_state[1] = msg.twist.twist.linear.y
        self.current_state[2] = msg.twist.twist.angular.z

    def ackermann_cb(self, msg):
        self.current_state[3] = msg.drive.steering_angle

    def collect_data(self):
        """
        Collects data during simulation.

        Adds the current state to the data array and updates the counter.
        Closes the progress bar and logs a message if data collection is complete.
        """
        if self.collection_done:
            return

        if self.current_state[0] > 1:  # Only collect data when the car is moving
            self.data = np.roll(self.data, -1, axis=0)
            self.data[-1] = self.current_state
            self.counter += 1
            self.pbar.update(1)

        if self.counter == self.timesteps + 1:
            self.collection_done = True
            self.pbar.close()
            self.timer.cancel()
            self.get_logger().info("Data collection completed.")

            # Run training and export in a separate thread to avoid blocking the executor
            thread = threading.Thread(target=self._train_and_export, daemon=True)
            thread.start()

    def _train_and_export(self):
        """Run NN training and CSV export, then shut down."""
        self.get_logger().info("Training neural network...")
        nn_train(self.data, self.racecar_version, self.save_LUT_name, self.plot_model)
        self.export_data_as_csv()
        self.get_logger().info("Training completed. Shutting down...")
        rclpy.shutdown()

    def run_nn_train(self):
        """
        Initiates training of the neural network using collected data.
        """
        self.get_logger().info("Training neural network...")
        nn_train(self.data, self.racecar_version, self.save_LUT_name, self.plot_model)


def main(args=None):
    rclpy.init(args=args)
    node = OnTrackSysId()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
