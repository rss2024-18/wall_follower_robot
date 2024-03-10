#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyControllerTest(Node):

    def __init__(self):
        super().__init__("safety_controller_test")

        self.publisher_ = self.create_publisher(AckermannDriveStamped, "/vesc/input/navigation", 10)
        timer_period = 1/500 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Process Lidar data
        """
        drive_msg = AckermannDriveStamped()

        drive_msg.header.stamp = self.get_clock().now().to_msg()
        #Change this speed to change the speed of the car during tests
        drive_msg.drive.speed = 2.0
        drive_msg.drive.steering_angle = 0.0
        self.get_logger().info(str("Driving 0"))

        self.publisher_.publish(drive_msg)


def main(args=None):
    
    rclpy.init(args=args)
    wall_follower = SafetyControllerTest()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()