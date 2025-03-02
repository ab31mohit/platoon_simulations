#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
from collections import deque
from tf_transformations import euler_from_quaternion
import numpy as np
import signal

class TrajectoryControl(Node):
    def __init__(self):
        super().__init__('trajectory_control')
        
        self.odom_sub_waffle1 = self.create_subscription(Odometry, '/waffle1/odom', self.waffle1_odom_callback, 10)
        self.odom_sub_waffle2 = self.create_subscription(Odometry, '/waffle2/odom', self.waffle2_odom_callback, 10)
        self.cmd_pub_waffle2 = self.create_publisher(Twist, '/waffle2/cmd_vel', 10)
        
        self.waffle1_traj = deque(maxlen=1000)  # Stores Waffle1's trajectory with limited size
        self.waffle2_pos = None  # Stores current position of Waffle2
        self.waffle1_yaw = None  # Stores current yaw of Waffle1
        self.waffle2_yaw = None  # Stores current yaw of Waffle2
        
        self.initial_pos_waffle1 = None  # Initial position of Waffle1
        self.initial_pos_waffle2 = None  # Initial position of Waffle2
        self.last_waffle1_pos = None  # Track last known position of Waffle1
        
        self.kp_linear = 1.0  # Proportional gain for linear velocity
        self.kd_linear = 0.5  # Derivative gain for linear velocity
        self.kp_angular = 2.5  # Proportional gain for angular velocity
        self.kd_angular = 0.5  # Derivative gain for angular velocity
        
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        
        self.trajectory_initialized = False

    def waffle1_odom_callback(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.waffle1_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        if self.initial_pos_waffle1 is None:
            self.initial_pos_waffle1 = (x, y)  # Store Waffle1's initial position
            self.last_waffle1_pos = (x, y)
            self.initialize_trajectory_if_ready()
            # print('\n[Updating waffle1 trajectory as it moves:]')
        
        if self.trajectory_initialized and self.distance(x, y, *self.last_waffle1_pos) > 0.01:  # Only update if position changes
            # print((x,y))
            self.waffle1_traj.append((x, y))
            self.last_waffle1_pos = (x, y)
        
    def waffle2_odom_callback(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.waffle2_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        if self.initial_pos_waffle2 is None:
            self.initial_pos_waffle2 = (x, y)
            self.initialize_trajectory_if_ready()
        
        self.waffle2_pos = (x, y)
        
        if self.trajectory_initialized:

            self.follow_trajectory()

    def initialize_trajectory_if_ready(self):
        if self.initial_pos_waffle1 is not None and self.initial_pos_waffle2 is not None:
            self.initialize_trajectory()

    def initialize_trajectory(self):
        num_waypoints = 20  # Adjust as needed
        initial_waypoints = self.generate_straight_line_waypoints(self.initial_pos_waffle2, self.initial_pos_waffle1, num_waypoints)
        print('[Initial Trajectory waypoints:]')

        for waypoint in initial_waypoints:
            print(waypoint)
            self.waffle1_traj.append(waypoint)
        
        self.trajectory_initialized = True
        self.get_logger().info("Trajectory initialized")

    def generate_straight_line_waypoints(self, start, end, num_points):
        x_start, y_start = start
        x_end, y_end = end
        x_points = np.linspace(x_start, x_end, num_points)
        # print('x_points:\n')
        # print(x_points)
        
        y_points = np.linspace(y_start, y_end, num_points)
        # print('\ny_points:\n')
        # print(y_points)

        return [(x, y) for x, y in zip(x_points, y_points)]

    def follow_trajectory(self):
        if len(self.waffle1_traj) < 2:
            return  # Wait for enough trajectory points
        
        target_x, target_y = self.waffle1_traj[0]  # Get the earliest stored position
        current_x, current_y = self.waffle2_pos
        
        if self.distance(current_x, current_y, target_x, target_y) > 0.05:
            cmd = self.compute_velocity(current_x, current_y, target_x, target_y, self.waffle2_yaw)
            self.cmd_pub_waffle2.publish(cmd)
        
        if self.distance(current_x, current_y, target_x, target_y) < 0.1:
            self.waffle1_traj.popleft()  # Remove old points once reached
    
    def compute_velocity(self, current_x, current_y, target_x, target_y, target_yaw):
        cmd = Twist()
        
        dx = target_x - current_x
        dy = target_y - current_y
        
        desired_yaw = math.atan2(dy, dx)
        
        angular_error = desired_yaw - target_yaw
        linear_error = math.sqrt(dx**2 + dy**2)
        
        d_linear = linear_error - self.prev_linear_error
        d_angular = angular_error - self.prev_angular_error
        
        cmd.linear.x = self.kp_linear * linear_error + self.kd_linear * d_linear
        cmd.angular.z = self.kp_angular * angular_error + self.kd_angular * d_angular
        
        self.prev_linear_error = linear_error
        self.prev_angular_error = angular_error
        
        return cmd
    
    @staticmethod
    def distance(x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def shutdown(self):
        self.get_logger().info('Shutting down. Stopping Waffle2...')
        stop_cmd = Twist()
        self.cmd_pub_waffle2.publish(stop_cmd)
        self.destroy_node()

# def main():
#     rclpy.init()
#     node = PlatoonControl()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryControl()
    
    def signal_handler(sig, frame):
        node.shutdown()
        rclpy.shutdown()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
