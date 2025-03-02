#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')
        
        # Subscribers
        self.create_subscription(Twist, '/waffle1/cmd_vel', self.waffle1_vel_callback, 10)
        self.create_subscription(Twist, '/waffle2/cmd_vel', self.waffle2_vel_callback, 10)
        self.create_subscription(Float64, '/platoon_control/waffle1_to_waffle2/current_distance', self.current_distance_callback, 10)
        self.create_subscription(Float64, '/platoon_control/waffle1_to_waffle2/desired_distance', self.desired_distance_callback, 10)
        self.create_subscription(Path, '/waffle1/robot_trajectory', self.waffle1_trajectory_callback, 10)
        self.create_subscription(Path, '/waffle2/robot_trajectory', self.waffle2_trajectory_callback, 10)
        
        # Data storage
        self.time = []
        self.waffle1_linear_vel = []
        self.waffle1_angular_vel = []
        self.waffle2_linear_vel = []
        self.waffle2_angular_vel = []
        self.current_distance = []
        self.desired_distance = []
        self.waffle1_trajectory_x = []
        self.waffle1_trajectory_y = []
        self.waffle2_trajectory_x = []
        self.waffle2_trajectory_y = []
        
        # Matplotlib setup
        plt.ion()
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 15))
        self.fig.suptitle('Platoon Control Visualization')
        
        # Initialize plots
        self.vel_lines = [self.ax1.plot([], [], label=label)[0] for label in ['Waffle1 Linear', 'Waffle1 Angular', 'Waffle2 Linear', 'Waffle2 Angular']]
        self.distance_lines = [self.ax2.plot([], [], label=label)[0] for label in ['Current Distance', 'Desired Distance']]
        self.trajectory_lines = [self.ax3.plot([], [], label=label)[0] for label in ['Waffle1 Trajectory', 'Waffle2 Trajectory']]
        
        for ax in (self.ax1, self.ax2, self.ax3):
            ax.legend()
            ax.grid(True)
        
        self.ax1.set_ylabel('Velocity')
        self.ax2.set_ylabel('Distance')
        self.ax3.set_xlabel('X')
        self.ax3.set_ylabel('Y')
        
        self.start_time = self.get_clock().now().to_msg().sec
        
    def waffle1_vel_callback(self, msg):
        self.update_data(self.waffle1_linear_vel, msg.linear.x)
        self.update_data(self.waffle1_angular_vel, msg.angular.z)
        
    def waffle2_vel_callback(self, msg):
        self.update_data(self.waffle2_linear_vel, msg.linear.x)
        self.update_data(self.waffle2_angular_vel, msg.angular.z)
        
    def current_distance_callback(self, msg):
        self.update_data(self.current_distance, msg.data)
        
    def desired_distance_callback(self, msg):
        self.update_data(self.desired_distance, msg.data)
        
    def waffle1_trajectory_callback(self, msg):
        self.waffle1_trajectory_x = [pose.pose.position.x for pose in msg.poses]
        self.waffle1_trajectory_y = [pose.pose.position.y for pose in msg.poses]
        
    def waffle2_trajectory_callback(self, msg):
        self.waffle2_trajectory_x = [pose.pose.position.x for pose in msg.poses]
        self.waffle2_trajectory_y = [pose.pose.position.y for pose in msg.poses]
        
    def update_data(self, data_list, value):
        current_time = self.get_clock().now().to_msg().sec - self.start_time
        if len(self.time) == 0 or current_time > self.time[-1]:
            self.time.append(current_time)
            data_list.append(value)
            if len(self.time) > 100:  # Keep only last 100 points
                self.time.pop(0)
                data_list.pop(0)
    
    def update_plot(self, frame):
        # Update velocity plot
        for line, data in zip(self.vel_lines, [self.waffle1_linear_vel, self.waffle1_angular_vel, self.waffle2_linear_vel, self.waffle2_angular_vel]):
            line.set_data(self.time, data)
        self.ax1.relim()
        self.ax1.autoscale_view()
        
        # Update distance plot
        for line, data in zip(self.distance_lines, [self.current_distance, self.desired_distance]):
            line.set_data(self.time, data)
        self.ax2.relim()
        self.ax2.autoscale_view()
        
        # Update trajectory plot
        self.trajectory_lines[0].set_data(self.waffle1_trajectory_x, self.waffle1_trajectory_y)
        self.trajectory_lines[1].set_data(self.waffle2_trajectory_x, self.waffle2_trajectory_y)
        self.ax3.relim()
        self.ax3.autoscale_view()
        
        return self.vel_lines + self.distance_lines + self.trajectory_lines

def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()
    
    ani = FuncAnimation(node.fig, node.update_plot, interval=100, blit=True)
    plt.show(block=True)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
