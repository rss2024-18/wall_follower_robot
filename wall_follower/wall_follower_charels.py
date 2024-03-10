#!/usr/bin/env python3
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
		
        self.subscription = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC,
            self.listener_callback,
            20)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 20)

        self.WALL_TOPIC = "/wall"
        self.line_pub = self.create_publisher(Marker, self.WALL_TOPIC, 1)

        self.kp = 0.8
        self.kd = 1.4
        self.ki = 0
        self.last_error = 0
        # self.prev_heading = 0
        # self.total_error = 0

    # TODO: Write your callback functions here    
        
    def listener_callback(self, msg):
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        ranges = np.clip(np.array(msg.ranges), msg.range_min, msg.range_max)
        lower, upper = min(self.SIDE*math.pi/6, self.SIDE*1/2*math.pi), max(self.SIDE*math.pi/6, self.SIDE*1/2*math.pi)
        li, ui = int((lower - msg.angle_min) / msg.angle_increment), int((upper - msg.angle_min) / msg.angle_increment)
        bounded = min(max(li + np.argmin(ranges[li:ui]) - self.SIDE * 22, 0), len(ranges)-1)
        li_new = min(li + np.argmin(ranges[li:ui]), bounded)
        ui_new = max(li + np.argmin(ranges[li:ui]), bounded)
        li, ui = li_new, ui_new
        ranges_sliced = ranges[li:ui]
        angles_sliced = np.linspace(math.pi/2 - (li*msg.angle_increment - msg.angle_min), math.pi/2 - (ui*msg.angle_increment - msg.angle_min), ui-li)
        x, y = ranges_sliced * np.cos(angles_sliced), ranges_sliced * np.sin(angles_sliced)

        A = np.vstack([x, np.ones(len(x))]).T
        m, b = np.linalg.lstsq(A, y)[0]
        y_wall = m * x + b
        VisualizationTools.plot_line(-x, y_wall, self.line_pub, frame="/laser")

        wall_end1, wall_end2, robot = np.array([x[0], y[0]]), np.array([x[-1], y[-1]]), np.array([0, 0])
        distance = np.linalg.norm(np.cross(wall_end2 - wall_end1, wall_end1 - robot)) / np.linalg.norm(wall_end2 - wall_end1)
        error = np.arcsin(max(min((distance - self.DESIRED_DISTANCE) / (self.VELOCITY * 0.05), 1), -1)) # self.SIDE * (distance - self.DESIRED_DISTANCE)
        # if (ranges[int(len(ranges)/2)] < 1.6 * self.DESIRED_DISTANCE):
        #     error = (1.6 * self.DESIRED_DISTANCE - ranges[int(len(ranges)/2)])

        command = AckermannDriveStamped()
        command.header = msg.header
        command.header.stamp = self.get_clock().now().to_msg()
        command.drive.steering_angle = self.SIDE * (self.kp * error + self.kd * (error - self.last_error)) #  + self.ki * self.total_error
        if (distance > 1.2 * self.DESIRED_DISTANCE and abs(m) >= 1.0):
            error = 0
            command.drive.steering_angle = -self.SIDE * math.pi / 6
        command.drive.steering_angle_velocity = 0.0
        command.drive.speed = self.VELOCITY # / np.cos(command.drive.steering_angle)
        command.drive.acceleration = 0.0
        command.drive.jerk = 0.0
        self.get_logger().info('angle_min: "%s"' % msg.angle_min)
        self.get_logger().info('angle_max: "%s"' % msg.angle_max)
        self.publisher.publish(command)

        # self.last_error = error
        # self.prev_heading = self.drive.steering_angle
        # self.total_error += error





def main():
    
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
