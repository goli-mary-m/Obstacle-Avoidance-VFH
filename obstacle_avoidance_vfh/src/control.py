#!/usr/bin/python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from obstacle_avoidance_vfh.msg import PolarHistogram
from math import radians, atan2, degrees
import numpy as np
from scipy.signal import argrelextrema
import matplotlib.pyplot as plt


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

        # sectors
        self.angular_resolution = 5
        self.n_sectors = int(360/self.angular_resolution)

        self.threshold = 0.6
        self.s_max = 15

        # starting point and target point
        self.x_src = 0
        self.y_src = 0
        self.x_des = 7
        self.y_des = 13

        self.target_sector = None

        # pid controller for angular speed
        k_p = 0.5
        k_i = 0
        k_d = 0
        self.pid_angular = PIDController("angular", k_p, k_i, k_d)

        self.angular_speed = 0
        self.linear_speed = 0.1
         
        
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


    def find_target_sector(self):

        robot_x, robot_y = self.get_position()

        yaw = self.get_heading()
        goal_angle = atan2((self.y_des-robot_y), (self.x_des-robot_x))		
        rotation_angle = abs(goal_angle - yaw)

        if(goal_angle - yaw >= 0): # target -> left
            target_angle = degrees(rotation_angle)
        else: # target -> right
            target_angle = 360 - degrees(rotation_angle)

        for k in range(0, self.n_sectors):
            sector_start = self.angular_resolution * (k)
            sector_end   = self.angular_resolution * (k + 1)   

            if(sector_start <= target_angle < sector_end):
                self.target_sector = k
                break


    def find_angle(self):

        valley_indices = argrelextrema(np.array(self.histogram), np.less)[0]
        print("valley: ", valley_indices)

        candidate_vallies = {}
        for i in range(len(self.histogram)):
            if(self.histogram[i] <= self.threshold):
                candidate_vallies[i] = self.histogram[i]
        print("candidate: ", candidate_vallies)  

        # check if target is in a valley or not
        if self.target_sector in candidate_vallies.keys():

            # move to the middle of the valley

            sector_start = self.target_sector * self.angular_resolution
            sector_end   = (self.target_sector + 1) * self.angular_resolution
            middle_angle = (sector_start + sector_end) / 2

            if(middle_angle <= 180):
                direction = +1
                angle = direction * middle_angle           
            else:
                direction = -1
                angle = direction * (360 - middle_angle)
                
            print("angle: ", angle)
            return angle    

        else:

            # find nearest candidate valley to target sector

            if(len(candidate_vallies) != 0):

                nearest_right_sector = min(candidate_vallies.keys(), key=lambda k: abs(k-self.target_sector))
                nearest_left_sector  = min(candidate_vallies.keys(), key=lambda k: abs(k-self.target_sector-self.n_sectors))
                nearest_sector = min(nearest_right_sector, nearest_left_sector)

                k_n = nearest_sector
                print("k_n: ", k_n)

                # find the number of consecutive sectors with PODs below the threshold
                if(k_n > self.target_sector):
                    step = +1
                else:
                    step = -1 

                consecutive_sectors = []
                curr_sector = k_n
                for k in range(0, self.n_sectors):
                    if(curr_sector in candidate_vallies.keys() or (curr_sector+self.n_sectors) in candidate_vallies.keys()):
                        consecutive_sectors.append(curr_sector)
                        curr_sector += step
                    else:
                        break  
                valley_size = len(consecutive_sectors)  
                print("valley_size: ", valley_size) 


                # find k_f (far border) based on type of valley (wide or narrow) 
                if(valley_size >= self.s_max): # wide
                    print("wide")
                    k_f = consecutive_sectors[self.s_max-1]
                else: # narrow
                    print("narrow")
                    k_f = consecutive_sectors[-1] # last element of array
                    
                print("k_f: ", k_f)

                # find angle
                if(k_f >= self.n_sectors/2):
                    k_f -= self.n_sectors
                    print("updated k_f: ", k_f)
                if(k_n >= self.n_sectors/2):
                    k_n -= self.n_sectors   
                    print("updated k_n: ", k_n)

                angle = (k_f + k_n)/2 * self.angular_resolution   
                print("angle: ", angle)  

                return angle   
                    

    def run(self):

        # create histogram plot
        fig, ax = plt.subplots()
        ax.set_ylim(0, 1)
        for i in range(len(self.histogram)):
                ax.bar(i, self.histogram[i], color='green')
        
        while not rospy.is_shutdown():

            # update histogram plot
            ax.cla()
            for i in range(len(self.histogram)):
                ax.bar(i, self.histogram[i], color='green')
            ax.set_ylim(0, 1)
            plt.pause(0.5)
            plt.draw()

            twist = Twist()
            twist.linear.x = 0.1
            self.cmd_publisher.publish(twist)

            # find sector of target point
            self.find_target_sector()
            print("target_sector: ", self.target_sector)

            # process histogram and find angle
            angle = self.find_angle() 
            
            if(angle != None):
            
                # update angular speed
                error_angular = radians(angle) 
                self.angular_speed = self.pid_angular.update(error_angular)

                print("angular speed: ", round(self.angular_speed, 3), "\n")

                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = self.angular_speed
                self.cmd_publisher.publish(twist) 

            print()
        

if __name__ == "__main__":

    controller = Controller()
    controller.run()