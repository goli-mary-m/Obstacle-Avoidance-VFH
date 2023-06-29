#!/usr/bin/python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from obstacle_avoidance_vfh.msg import PolarHistogram
from math import radians
import numpy as np
from scipy.signal import argrelextrema
import matplotlib.pyplot as plt
import time


class PIDController():

    def __init__(self,type, k_p, k_i, k_d) -> None:

        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        self.prev_error = 0
        self.sum_error = 0

        self.dt = 0.005
        self.type = type


    def update(self, error):

        # calculate proportional term
        P = self.k_p * error

        # calculate integral term
        self.sum_error += error * self.dt
        I = self.k_i * self.sum_error

        # calculate derivative term
        D = self.k_d * (error - self.prev_error)
        self.prev_error = error

        # return total control output
        control_output = P + I + D
        return control_output  



class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("control_node" , anonymous=False)

        self.cmd_publisher = rospy.Publisher('cmd_vel' , Twist , queue_size=10)
        self.vfh_subscriber = rospy.Subscriber('polar_histogram', PolarHistogram, self.vfh_callback)
        self.histogram = []

        # starting point and target point
        self.x_src = 0
        self.y_src = 0
        self.x_des = 7
        self.y_des = 13

        # pid controller for angular speed
        k_p = 1
        k_i = 0
        k_d = 0
        self.pid_angular = PIDController("angular", k_p, k_i, k_d)

        self.angular_speed = 0
        self.linear_speed = 0.3
         
        
    def vfh_callback(self, msg: PolarHistogram):

        self.histogram = msg.histogram
    

    def get_heading(self):
        
        msg = rospy.wait_for_message("odom" , Odometry)
        orientation = msg.pose.pose.orientation
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw


    def get_position(self):

        msg = rospy.wait_for_message("odom", Odometry)
        position = msg.pose.pose.position
        robot_x = position.x
        robot_y = position.y

        return robot_x, robot_y     
    

    def find_angle(self):

        # TODO:
        # - process polar histogram and find angle
        pass
        

    def run(self):

        # create histogram plot
        fig, ax = plt.subplots()
        n, bins, patches = ax.hist(self.histogram, bins=72)
        
        while not rospy.is_shutdown():

            # update histogram plot
            ax.cla()
            n, bins, patches = ax.hist(self.histogram, bins=72)
            plt.pause(0.5)
            plt.draw()

            twist = Twist()
            twist.linear.x = 0.1
            self.cmd_publisher.publish(twist)

            # self.find_angle() 

            # robot_x, robot_y = self.get_position()
            
            # # update angular speed
            # error_angular = self.find_angle(robot_x, robot_y) 
            # self.angular_speed = self.pid_angular.update(error_angular)

            # print("linear: ", round(self.linear_speed, 3), " | angular: ", round(self.angular_speed, 3), "\n")

            # twist = Twist()
            # twist.linear.x = self.linear_speed
            # twist.angular.z = self.angular_speed
            # self.cmd_publisher.publish(twist) 
        

if __name__ == "__main__":

    controller = Controller()
    controller.run()