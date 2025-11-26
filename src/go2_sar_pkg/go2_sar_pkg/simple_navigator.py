#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        
        # 1. Declare Parameters (Arguments you pass in command line)
        self.declare_parameter('target_x', 1.0) # Default 1m forward
        self.declare_parameter('target_y', 0.0)
        
        # 2. Get the Goal
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        
        # 3. Subscribe to Unitree's Internal SLAM Odometry
        # This tells us where we are relative to where we started
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.odom_callback, 10)
        
        # 4. Publisher to send commands to your Bridge
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_reached = False
        
        self.get_logger().info(f"Navigator Started! Goal: x={self.target_x}, y={self.target_y}")

    def odom_callback(self, msg):
        if self.goal_reached:
            return

        # 1. Update Position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convert Quaternion to Yaw (Rotation around Z)
        q = msg.pose.pose.orientation
        # Standard formula for yaw from quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 2. Run Control Logic
        self.move_to_goal()

    def move_to_goal(self):
        # Calculate distance to goal
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Calculate the angle we NEED to be facing
        target_angle = math.atan2(dy, dx)
        
        # Calculate the error (difference between current heading and target)
        angle_diff = target_angle - self.current_yaw
        
        # Normalize angle to [-pi, pi] so it doesn't spin the long way
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi

        cmd = Twist()

        # --- LOGIC: Point and Shoot ---
        
        # A. Stop if close enough (10cm tolerance)
        if distance < 0.1:
            self.get_logger().info("GOAL REACHED!")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.goal_reached = True
            return

        # B. If we are facing the wrong way (more than 30 degrees off), TURN first
        if abs(angle_diff) > 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8 * angle_diff # Turn speed proportional to error
            # Clamp max turn speed
            cmd.angular.z = max(min(cmd.angular.z, 0.8), -0.8)
            self.get_logger().info(f"Turning... Angle Error: {angle_diff:.2f}")

        # C. If we are mostly facing the target, DRIVE
        else:
            cmd.linear.x = 0.4 # Walk forward at 0.4 m/s
            cmd.angular.z = 1.0 * angle_diff # Correct heading while walking
            self.get_logger().info(f"Walking... Dist: {distance:.2f}")

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop on Ctrl+C
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
