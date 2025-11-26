#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request
import json

class CmdVelToGo2(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_go2_bridge')

        # 1. Subscribe to standard ROS movement commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

        # 2. Publisher to talk to Unitree Sport Service (SDK2)
        self.req_pub = self.create_publisher(Request, '/api/sport/request', 10)

        self.get_logger().info("Bridge Started: /cmd_vel -> Unitree Go2 API")

    def listener_callback(self, msg):
        # Extract velocities
        # x = Forward/Backward (max ~1.0 m/s)
        # y = Strafe Left/Right (max ~0.5 m/s)
        # z = Turn Left/Right (max ~1.0 rad/s)
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Construct the JSON command Unitree expects
        # API ID 1004 is "Move"
        req = Request()
        req.header.identity.api_id = 1004 

        # The Go2 expects the parameter to be a JSON string: {"x": 0.0, "y": 0.0, "z": 0.0}
        command_data = {'x': vx, 'y': vy, 'z': wz}
        req.parameter = json.dumps(command_data)

        # Publish
        self.req_pub.publish(req)

        # Debug print (Optional: comment out if too spammy)
        # self.get_logger().info(f"Sent: {command_data}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToGo2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
