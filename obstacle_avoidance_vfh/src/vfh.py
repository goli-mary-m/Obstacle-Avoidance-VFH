#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt


class ActiveCell:

    def __init__(self, i, j) -> None:

        self.i = i
        self.j = j
        self.cell_size = 1

        # coordinates for center of cell
        self.center_x = i + (self.cell_size/2)
        self.center_y = j + (self.cell_size/2)

        # magnitude of cell: m_ij = ((c_ij)**2) * (a - (b*d_ij))
        self.c_ij = 0
        self.d_ij = 0
        self.m_ij = 0


    def calculate_magnitude(self, a, b):

        # calculate m_ij
        self.m_ij = (self.c_ij**2) * (a - (b*self.d_ij))


    def calculate_betha(self, robot_x, robot_y):

        # calculate direction from active cell to robot
        betha = atan2(self.center_x-robot_x, self.center_y-robot_y)
        return betha



class Sector:

    def __init__(self, k, angular_resolution) -> None:
        
        self.k = k

        # start and end of interval
        self.sector_start = angular_resolution * (self.k)
        self.sector_end   = angular_resolution * (self.k + 1)

        # array of cells in sector
        self.cells = []

        # polar obstacle density of sector: h_k = sum(m_ij for sector cells) 
        self.h_k = 0
        self.h_k_prime = 0


    def add_cell(self, cell_i, cell_j):

        cell = (cell_i, cell_j)
        self.cells.append(cell)


    def set_polar_obstacle_density(self, h_k):

        self.h_k = h_k


    def set_smoothed_polar_obstacle_density(self, h_k_prime):

        self.h_k_prime = h_k_prime    



class VFH:
    
    def __init__(self) -> None:
        
        rospy.init_node("vfh" , anonymous=False)
        
        # starting point and target point
        self.x_src = 0
        self.y_src = 0
        self.x_des = 7
        self.y_des = 13

        # constants
        self.a = 1
        self.b = 0.25
        self.l = 2

        # active window
        self.window_size = round((sqrt(2)*(self.a/self.b)) + 1)

        self.window_x_min = 0
        self.window_x_max = 0
        self.window_y_min = 0
        self.window_y_max = 0

        self.active_cells = []

        self.angular_resolution = 5
        self.n_sectors = int(360/self.angular_resolution)
        self.sectors = []

        # vector field histogram array
        self.vfh_arr = []
    

    def get_position(self):

        msg = rospy.wait_for_message("odom", Odometry)
        position = msg.pose.pose.position
        robot_x = position.x
        robot_y = position.y

        return robot_x, robot_y        
    

    def find_active_cells(self, robot_x, robot_y):

        i = self.window_x_min
        j = self.window_y_min
        for cnt_i in range(0, self.window_size):

            cells = []
            for cnt_j in range(0, self.window_size):

                # create new ActiveCell object with i, j
                active_cell = ActiveCell(i, j)
                cells.append(active_cell)

                # find index of sector (k) for current cell
                betha = active_cell.calculate_betha(robot_x, robot_y)
                k = int(betha/self.angular_resolution)

                # add current cell to sector k
                self.sectors[k].add_cell(i, j)

                j += 1

            self.active_cells.append(cells)
            i += 1   


    def process_laser_data(self):

        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        laser_rng = laser_data.ranges  

        # TODO:
        # - update cij, dij for each active_cell  


    def get_active_cell(self, i, j):

        for cnt_i in range(0, self.window_size):
                for cnt_j in range(0, self.window_size):
                    curr_cell = self.active_cells[cnt_i][cnt_j]
                    if(curr_cell.i == i and curr_cell.j == j):
                        return curr_cell
                    
        return None                


    def run(self):
        
        while not rospy.is_shutdown():

            robot_x, robot_y = self.get_position()

            self.window_x_min = robot_x - (self.window_size-1)/2
            self.window_x_max = robot_x + (self.window_size-1)/2
            self.window_y_min = robot_y - (self.window_size-1)/2
            self.window_y_max = robot_y + (self.window_size-1)/2

            # print("x_min: ", self.window_x_min)
            # print("x_max: ", self.window_x_max)
            # print("y_min: ", self.window_y_min)
            # print("y_max: ", self.window_y_max)

            # determine sectors
            for k in range(0, self.n_sectors):
                self.sectors.append(Sector(k, self.angular_resolution))

            # find active cells and map them to corresponding sectors
            self.find_active_cells(robot_x, robot_y)

            # process LaserScan data and find c_ij, d_ij for each active cell
            self.process_laser_data()

            # calculate magnitude (m_ij) for all active cells
            for cnt_i in range(0, self.window_size):
                for cnt_j in range(0, self.window_size):
                    self.active_cells[cnt_i][cnt_j].calculate_magnitude(self.a, self.b)

            # calculate polar obstacle density (h_k) of all sectors
            for k in range(0, self.n_sectors):
                h_k = 0
                for cell_i, cell_j in self.sectors[k].cells:
                    cell = self.get_active_cell(cell_i, cell_j)
                    if(cell != None):
                        h_k += cell.m_ij
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
                h_k_prime += self.sectors[k-1] + self.sectors[k+1]
                h_k_prime /= 2*self.l + 1            
                        
                self.sectors[k].set_smoothed_polar_obstacle_density(h_k_prime)        
                self.vfh_arr.append(h_k_prime)
     


if __name__ == "__main__":

    vfh = VFH() 
    vfh.run()