#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class SteeringJoystickController(Node):
    def __init__(self):
        super().__init__('steering_joystick_controller')

        # Parameters
        self.declare_parameter('max_angle', 3.14)      # radians (max steering lock)
        self.declare_parameter('deadzone', 0.05)      # axis deadzone
        self.declare_parameter('axis', 0)             # which axis (0 = left-right)
        self.declare_parameter('invert', False)       # invert direction?

        self.max_angle = self.get_parameter('max_angle').value
        self.deadzone = self.get_parameter('deadzone').value
        self.axis = self.get_parameter('axis').value
        self.invert = -1 if self.get_parameter('invert').value else 1

        # Publisher to the position controller
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',  # default topic for ForwardCommandController
            10
        )

        # Subscriber to joystick
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info('Steering joystick controller started')

    def joy_callback(self, msg: Joy):
        # Read the desired axis (default: left-right is axis 0)
        raw = msg.axes[self.axis] * self.invert

        # Apply deadzone
        if abs(raw) < self.deadzone:
            raw = 0.0

        # Scale to steering angle (clamp to [-1, 1])
        cmd = raw
        if cmd > 1.0:
            cmd = 1.0
        elif cmd < -1.0:
            cmd = -1.0
        cmd *= self.max_angle

        # Publish as Float64MultiArray (single element)
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [cmd]
        self.cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringJoystickController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()