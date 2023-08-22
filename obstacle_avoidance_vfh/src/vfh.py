#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from obstacle_avoidance_vfh.msg import PolarHistogram
from math import sqrt, cos, sin
import numpy as np


class Sector:

    def __init__(self, k, angular_resolution) -> None:
        
        self.k = k

        # start and end of interval
        self.start = angular_resolution * (self.k)
        self.end   = angular_resolution * (self.k + 1)

        # polar obstacle density
        self.h_k = 0
        self.h_k_prime = 0


    def set_polar_obstacle_density(self, h_k):
        self.h_k = h_k


    def set_smoothed_polar_obstacle_density(self, h_k_prime):
        self.h_k_prime = h_k_prime    



class VFH:
    
    def __init__(self) -> None:
        
        rospy.init_node("vfh_node" , anonymous=False)

        # constants
        self.a = 1
        self.b = 0.25
        self.l = 2

        self.c_array = np.zeros(360)
        self.d_array = np.zeros(360)
        self.m_array = np.zeros(360)

        # active window
        self.window_size = round((sqrt(2)*(self.a/self.b)) + 1)

        self.window_x_min = 0
        self.window_x_max = 0
        self.window_y_min = 0
        self.window_y_max = 0

        # sectors
        self.angular_resolution = 5
        self.n_sectors = int(360/self.angular_resolution)
        self.sectors = []

        # vector field histogram array 
        self.vfh_arr = []
        self.polar_histogram = PolarHistogram()
        self.vfh_publisher = rospy.Publisher('polar_histogram', PolarHistogram, queue_size=10)
    

    def get_position(self):

        msg = rospy.wait_for_message("odom", Odometry)
        position = msg.pose.pose.position
        robot_x = position.x
        robot_y = position.y

        return robot_x, robot_y        


    def process_laser_data(self):

        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        laser_rng = laser_data.ranges  

        for i in range(0, 360):

            angle = i
            distance = laser_rng[i]

            if(self.is_in_window(angle, distance) == True):
                self.c_array[i] = 1
                self.d_array[i] = distance
                self.m_array[i] = self.calculate_magnitude(angle)
                   

    def is_in_window(self, angle, distance):

        # find d_max if active window is square
        # robot_distance = (self.window_size-1)/2
        # if((angle in range(45, 135)) or (angle in range(225, 315))):
        #     d_max = robot_distance/abs(sin(angle))
        # else:   
        #     d_max = robot_distance/abs(cos(angle))

        # find d_max if active window is circular
        d_max = self.a/self.b

        if(distance <= d_max):
            return True
        else:
            return False


    def calculate_magnitude(self, i):

        # magnitude = ((c**2) * (a - (b*d))
        return (self.c_array[i]**2) * (self.a - (self.b*self.d_array[i]))
    

    def run(self):
        
        while not rospy.is_shutdown():

            self.vfh_arr = []
            robot_x, robot_y = self.get_position()

            self.window_x_min = robot_x - (self.window_size-1)/2
            self.window_x_max = robot_x + (self.window_size-1)/2
            self.window_y_min = robot_y - (self.window_size-1)/2
            self.window_y_max = robot_y + (self.window_size-1)/2

            # determine sectors
            for k in range(0, self.n_sectors):
                self.sectors.append(Sector(k, self.angular_resolution))

            # process LaserScan data and find c, d, m for each angle between 0 and 360
            self.process_laser_data()

            # calculate polar obstacle density (h_k) of all sectors
            for k in range(0, self.n_sectors):
                h_k = 0
                for i in range(self.sectors[k].start, (self.sectors[k].end%360)+1):
                    h_k += self.m_array[i]
                self.sectors[k].set_polar_obstacle_density(h_k)  

            # apply smoothing function and calculate smoothed polar obstacle density (h_k_prime)
            for k in range(0, self.n_sectors):

                h_k_prime = 0
                # iterate over h_k values from index k-l+1 to k+l-1
                for k_prime in range(k-self.l+1, ((k+self.l-1)%self.n_sectors)+1):
                    if(k_prime == k):
                        h_k_prime += self.sectors[k_prime].h_k * self.l
                    else:
                        h_k_prime += self.sectors[k_prime].h_k * 2
                h_k_prime += self.sectors[k-self.l].h_k + self.sectors[(k+self.l)%self.n_sectors].h_k
                h_k_prime /= 2*self.l + 1            
                        
                self.sectors[k].set_smoothed_polar_obstacle_density(h_k_prime)        
                self.vfh_arr.append(h_k_prime)

            # normalize histogram
            # max_h = max(self.vfh_arr)
            # self.vfh_arr = [h/max_h for h in self.vfh_arr]    

            # publish polar histogram
            self.polar_histogram.histogram = self.vfh_arr
            self.vfh_publisher.publish(self.polar_histogram)    
     

if __name__ == "__main__":

    vfh = VFH() 
    vfh.run()