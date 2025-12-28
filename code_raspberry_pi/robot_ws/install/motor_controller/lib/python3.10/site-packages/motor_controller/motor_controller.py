#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        # Serial to Arduino Nano
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            self.get_logger().info("Serial opened: /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"Cannot open serial: {e}")
            self.ser = None

        # Maximum ticks / PWM sent to Arduino
        self.MAX_TICKS = 255
        # Distance between wheels (meters)
        self.L = 0.2  

    def listener_callback(self, msg):
        # Get linear and angular velocities
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive inverse kinematics
        vL = v - w*self.L/2
        vR = v + w*self.L/2

        # Map float velocities to integer ticks/PWM
        vL_ticks = int(vL * self.MAX_TICKS)
        vR_ticks = int(vR * self.MAX_TICKS)

        self.get_logger().info(f'Received cmd_vel: v={v:.2f}, w={w:.2f} -> vL={vL_ticks}, vR={vR_ticks}')

        # Send to Arduino in ROSArduinoBridge format: 'm arg1 arg2\r'
        if self.ser:
            command = f"m {vL_ticks} {vR_ticks}\r"
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent to Arduino: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
 
