#!/usr/bin/env python3
import numpy as np
import rclpy
import math
import random
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")
        #Declare attributes 
        self.laserScan = None
        self.steering_angle = 0.0 
        self.current_side = 0.0 
        self.prev_error = 0.0

        self.kp = 0.5
        self.ki = 0.0 
        self.kd = 0.3
        
        self.rwall_m = 0.0 
        self.rwall_c = 0.0
        self.lwall_m = 0.0
        self.lwall_c = 0.0

        self.lastm = 0.0
        self.lastlastm = 0.0

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
        self.WALL_TOPIC = "/wall"

	    # TODO: Initialize your publishers and subscribers here
        self.subscription = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)

        # a publisher for our line marker
        self.line_pub = self.create_publisher(Marker, self.WALL_TOPIC, 1)

        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0



    # TODO: Write your callback functions here  
    def listener_callback(self, msg):
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = 0.0 #self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.laserScan = msg

        #self.get_logger().info('HEARD ANYTHING')
    
    def timer_callback(self):
        right_range, middle_range, left_range, rmin_angle, mmin_angle, lmin_angle = self.slice_ranges()
        rmin_values, rmin_angles = self.closest_spot(right_range, rmin_angle, 70)
        lmin_values, lmin_angles = self.closest_spot(left_range, lmin_angle, 70)
        mmin_values, mmin_angles = self.closest_spot(middle_range, mmin_angle, 1)
        self.rwall_m, self.rwall_c, rx, ry = self.fit_wall(rmin_values, rmin_angles) #if wall on left c>0, if wall on right c<0, c = dist to wall
        self.lwall_m, self.lwall_c, lx, ly = self.fit_wall(lmin_values, lmin_angles)
        self.mwall_m, self.mwall_c, mx, my = self.fit_wall(mmin_values, mmin_angles)

        self.get_logger().info('minimum angle'+str(min(rmin_angle)))
        self.get_logger().info('max angle'+str(max(rmin_angle)))

        dist = self.slice_and_plot(self.laserScan)
        
        # #visualize line of side we are supposed to be on
        # if self.SIDE > 0: #left side
        #     #self.current_side = 1.0 
        #     #self.get_logger().info('LEFT LEFT LEFT LEFT LEFT LEFT LEFT LEFT LEFT ')
        #     x_line = np.linspace(min(lx), max(lx))
        #     y_line = self.lwall_m*x_line + self.lwall_c
        #     wall_dist = self.lwall_c
        #     better_wall_dist = self.get_wall_dist(self.lwall_m, self.lwall_c)
        #     m = self.lwall_m
        # else:
        #     #self.current_side = -1.0 
        #     #self.get_logger().info('RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGTH RIGHT')
        #     x_line = np.linspace(min(rx), max(rx))
        #     y_line = self.rwall_m*x_line + self.rwall_c
        #     wall_dist = self.rwall_c
        #     better_wall_dist = self.get_wall_dist(self.rwall_m, self.rwall_c)
        #     m = self.rwall_m

        #VisualizationTools.plot_line(x_line, y_line, self.line_pub, frame="/laser")

        error = abs(dist) - self.DESIRED_DISTANCE

        wall_close = np.mean(mmin_values)
        #if wall_close < 1.5: #check if theres a wall in front of you
            #self.get_logger().info('WALL WALL WALL WALL WALL WALL WALL WALL WALL WALL WALL')
            #self.get_logger().info('original error' + str(error))
         #   error = error - wall_close
            #self.get_logger().info('new error' + str(error))


        #self.get_logger().info("wall dist " + str(wall_dist))
        #self.get_logger().info("better wall dist " + str(wall_dist))
        #self.get_logger().info("error  " + str(error))
        #write drive instructions
        msg = AckermannDriveStamped()

        change = self.pid(error, self.prev_error)
        self.prev_error = error

        steer = change*self.SIDE

        if abs(steer) > 0.34 and steer > 0:
            steer = 0.34
        elif abs(steer) > 0.34 and steer < 0:
            steer = -0.34

        self.steering_angle = steer
        #self.get_logger().info("steer " + str(steer))
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.laserScan.header.frame_id
        #self.get_logger().info('array size'+str(len(self.laserScan.ranges)))
        msg.drive.speed = self.VELOCITY #* 1.3 * max(0.5, 1 - 0.1 * error ** 2)
        msg.drive.steering_angle = self.steering_angle
        self.publisher_.publish(msg)
        #self.get_logger().info('Steering Angle: "%s"' % msg.drive.steering_angle)
        self.i+=1
    
    def slice_and_plot(self, msg):
        ## slice
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

        ## plots
        A = np.vstack([x, np.ones(len(x))]).T
        m, b = np.linalg.lstsq(A, y)[0]
        y_wall = m * x + b
        VisualizationTools.plot_line(-x, y_wall, self.line_pub, frame="/laser")

        ## gets the distance
        wall_end1, wall_end2, robot = np.array([x[0], y[0]]), np.array([x[-1], y[-1]]), np.array([0, 0])
        distance = np.linalg.norm(np.cross(wall_end2 - wall_end1, wall_end1 - robot)) / np.linalg.norm(wall_end2 - wall_end1)
        return distance

    def pid(self, error, prev_error):
        change = self.kp*error + self.kd*(error-prev_error)
        return change #change in angle 
    
    def slice_ranges(self):
        #slice ranges into a right, left, and middle section
        ranges = np.array(self.laserScan.ranges)
        length = len(ranges)
        middle_len = int((np.pi/6) // self.laserScan.angle_increment)

        #regions for other slices 
        cutout = int(((-np.pi/2) - self.laserScan.angle_min) // self.laserScan.angle_increment)
        right_start = cutout
        right_end = (length - middle_len) // 2
        left_start = right_end + middle_len
        left_end = length-cutout

        # Extract left and right ranges
        left_range = ranges[left_start:left_end]
        right_range = ranges[right_start:right_end]
        middle_range = ranges[(right_end + 1):(left_start - 1)]

        #get min angles, this is the angle the first value of each range is at 
        rmin_ang = self.laserScan.angle_min
        mmin_ang = ((right_end + 1) * self.laserScan.angle_increment) + self.laserScan.angle_min
        lmin_ang = (left_start * self.laserScan.angle_increment) + self.laserScan.angle_min

        return (right_range, middle_range, left_range, rmin_ang, mmin_ang, lmin_ang)

    def closest_spot(self, range, min_angle, groups):
        ranges = np.array(range)
        ranges = self.clean_data(ranges, 0.2)
        # Find the indices of the start of each group consecutive values
        start_indices = np.arange(len(ranges) - (groups-1))

        # Calculate the average values for each group of consecutive values
        averages = np.array([np.mean(ranges[i:i+groups]) for i in start_indices])

        # Find the index of the group with the smallest average
        min_index = np.argmin(averages)
        # Get the indices of the 7 consecutive values with the smallest average
        min_indices = np.arange(start_indices[min_index], start_indices[min_index] + groups)

        # Get the mean of the smallest distance values 
        min_values = ranges[min_indices]
        min_angles = (min_indices * self.laserScan.angle_increment) + np.ones(len(min_indices))*min_angle

        return (min_values, min_angles)
    
    def clean_data (self, arr, threshold):
        """
            parses array arr and replaces values that are below threshold with the value immediatley next to them
            for removing noisy data in the lidar 
            arr must be a numpy array
        """
        # Create a boolean mask where values are less than the threshold
        mask = arr < threshold
        # Create an array containing the values that will be replaced
        replaced_values = arr[mask]
        #self.get_logger().info('replaced'+str(replaced_values))
        # Replace values below the threshold with the value next to it
        #arr[mask] = np.roll(arr, 1)[mask]
         # Replace values below the threshold with the input replacement value
        arr[mask] = self.laserScan.range_max
        return arr

    def polar2cart(self, points, angles):
        #transform the polar coordinates of the robot to a local cartesian frame 

        x = np.array(points * np.cos(angles))
        y = np.array(points * np.sin(angles))
        return (x, y)

    def fit_wall(self, min_values, min_angles):
      #using the closest part of the wall, create a line of best fit
        x, y = self.polar2cart(min_values, min_angles)
        A = np.vstack([x, np.ones(len(x))]).T
        m, c = np.linalg.lstsq(A, y, rcond=None)[0]
        return (m, c, x, y)


    def get_wall_dist(self, m, b):
        # Slope of perpendicular line
        perpendicular_slope = -1 / m

        # Intersection point
        x1 = -b / (m + 1/m)
        y1 = m * x1 + b

        # Length of line segment using Pythagorean theorem
        length = math.sqrt(x1**2 + y1**2)

        return length
    
    def slice_scan(self, ranges):
        """
        slice scan into managable pieces a
        """
        back_front_divider = int(len(ranges) * 16/33)
        front_middle_divider = int(len(ranges) * 16/33)
        total_length = int(len(ranges))
        bls = ranges[total_length - back_front_divider:total_length]
        ms = ranges[front_middle_divider:total_length - front_middle_divider]
        brs = ranges[0:back_front_divider]
        return {'brs': brs, "ms": ms, "bls" : bls}
    
    # def return_data(self, data, flag = 0):
    #     """
    #     Returns appropriate data slice for side and speed
    #     """
    #     data = self.slice_scan(data)
    #     if flag:
    #         return np.array(data["ms"])
    #     if self.SIDE == -1:
    #         return np.array(data["brs"])
    #     return np.flip(np.array(data["bls"]))

def main():
    
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
