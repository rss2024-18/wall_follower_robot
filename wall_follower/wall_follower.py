#!/usr/bin/env python3
import numpy as np
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):
    WALL_TOPIC = "/wall"
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

        timer_period = 0.05 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    # TODO: Write your callback functions here  
    def listener_callback(self, msg):
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.laserScan = msg
        #self.get_logger().info('HEARD ANYTHING')
    
    def timer_callback(self):
        right_range, middle_range, left_range, rmin_angle, mmin_angle, lmin_angle = self.slice_ranges()
        rmin_values, rmin_angles = self.closest_spot(right_range, rmin_angle, 15)
        lmin_values, lmin_angles = self.closest_spot(left_range, lmin_angle, 15)
        mmin_values, mmin_angles = self.closest_spot(middle_range, mmin_angle, 1)
        self.rwall_m, self.rwall_c, rx, ry = self.fit_wall(rmin_values, rmin_angles) #if wall on left c>0, if wall on right c<0, c = dist to wall
        self.lwall_m, self.lwall_c, lx, ly = self.fit_wall(lmin_values, lmin_angles)
        self.mwall_m, self.mwall_c, mx, my = self.fit_wall(mmin_values, mmin_angles)

        #visualize line of side we are supposed to be on
        if self.SIDE > 0: #left side
            #self.current_side = 1.0 
            #self.get_logger().info('LEFT LEFT LEFT LEFT LEFT LEFT LEFT LEFT LEFT ')
            x_line = np.linspace(min(lx), max(lx))
            y_line = self.lwall_m*x_line + self.lwall_c
            wall_dist = self.lwall_c
            better_wall_dist = self.get_wall_dist(self.lwall_m, self.lwall_c)
            m = self.lwall_m
        else:
            #self.current_side = -1.0 
            #self.get_logger().info('RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGTH RIGHT')
            x_line = np.linspace(min(rx), max(rx))
            y_line = self.rwall_m*x_line + self.rwall_c
            wall_dist = self.rwall_c
            better_wall_dist = self.get_wall_dist(self.rwall_m, self.rwall_c)
            m = self.rwall_m

        VisualizationTools.plot_line(x_line, y_line, self.line_pub, frame="/laser")

        error = abs(better_wall_dist) - self.DESIRED_DISTANCE

        wall_close = np.mean(mmin_values)
        if wall_close < 1.5: #check if theres a wall in front of you
            self.get_logger().info('WALL WALL WALL WALL WALL WALL WALL WALL WALL WALL WALL')
            self.get_logger().info('original error' + str(error))
            error = error - wall_close
            self.get_logger().info('new error' + str(error))


        #self.get_logger().info("wall dist " + str(wall_dist))
        #self.get_logger().info("better wall dist " + str(wall_dist))
        #self.get_logger().info("error  " + str(error))
        #write drive instructions
        msg = AckermannDriveStamped()
        msg.drive.speed = self.safety_controller(self.laserScan.ranges)

        change = self.pid(error, self.prev_error)
        self.prev_error = error

        steer = change*self.SIDE

        if abs(steer) > 0.34 and steer > 0:
            steer = 0.34
        elif abs(steer) > 0.34 and steer < 0:
            steer = -0.34

        self.steering_angle = steer
        self.get_logger().info("steer " + str(steer))

        msg.drive.steering_angle = self.steering_angle
        self.publisher_.publish(msg)
        #self.get_logger().info('Steering Angle: "%s"' % msg.drive.steering_angle)
        self.i+=1

    def pid(self, error, prev_error):
        change = self.kp*error + self.kd*(error-prev_error)
        return change #change in angle 
    
    def slice_ranges(self):
        #slice ranges into a right, left, and middle section
        ranges = np.array(self.laserScan.ranges)
        length = len(ranges)
        middle_len = 20

        #regions for other slices 
        right_start = 0
        right_end = (length - middle_len) // 2
        left_start = right_end + middle_len
        left_end = length

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
        #determine which side the wall is on by averaging which side is reading closer
        ranges = np.array(range)
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
    
    def safety_controller(self, msg):
        """
        Compute distance to front wall and stop the car if the wall is too close
        """
        distance_needed_to_stop = self.VELOCITY * 1/3
        front_data = np.array(self.return_data(msg.ranges, 1))
        middle_angles = np.array(np.cos(self.return_data(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment).tolist(), 1)))
        true_dist = np.multiply(middle_angles, front_data)
        if np.median(true_dist) < distance_needed_to_stop:
            return 0.0
        return self.VELOCITY
    
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
    
    def return_data(self, data, flag = 0):
        """
        Returns appropriate data slice for side and speed
        """
        data = self.slice_scan(data)
        if flag:
            return np.array(data["ms"])
        if self.SIDE == -1:
            return np.array(data["brs"])
        
        return np.flip(np.array(data["bls"]))

def main():
    
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()