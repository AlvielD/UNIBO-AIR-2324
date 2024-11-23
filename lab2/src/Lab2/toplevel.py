"""
============== UniBo: AI and Robotics 2024 ==============
Base code: top level execution loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
import time, math
from robot_gwy import RobotGateway
from observer import Observer, Gridmap, Occupancy, Sonar_Model
from controller import Controller
import matplotlib.pyplot as plt
import rospy as ros
import numpy as np
from pgm_reader import Reader

class TopLevelLoop:
    """
    Top level execution loop
    """

    def __init__(self,
                 goal = (0, 0),         # goal position
                 tcycle = 0.1 ,         # cycle time, in sec
                 debug = 0              # debug level, use to decide to print debug info
                 ):
        self.tcycle = tcycle
        self.goal = goal
        self.debug = debug

        self.robot_gwy = RobotGateway()
        self.observer = Observer()
        self.controller = Controller()

        self.est_his = []
        self.gt_his = []

    def step(self):
        """
        Execute one step of this instance of the Top Level loop
        debug level can be used to selectively print debug information
        """
        wl, wr = self.robot_gwy.get_wheel_encoders()      # read sensors (wheel rotations)
        if self.debug > 0: gt_pose = self.robot_gwy.get_ground_truth_values()

        sdata = self.robot_gwy.get_sonar_data()             # read exteroceptive sensors (sonars)
        if self.debug > 1:
            print('sonars = {}'.format([s[1] for s in sdata]))

        mypose = self.observer.update_pose(wl, wr)        # estimate state (robot's pose)

        self.gridmap.update_grid(sdata, mypose)           # estimate environment state (occupancy grid)

        vlin, vrot = self.controller.compute_ctr(mypose)  # decide action (robot's vel)
        self.robot_gwy.set_vel_values(vlin, vrot)         # send control action
        if self.debug > 0:
            #self.robot_gwy.set_vel_values(0.0, 0.1)
            #print('Wheel position = ({:.2f}, {:.2f})'.format(wl, wr))

            # Store estimated and ground truth positions (x, y)
            self.est_his.append(mypose)
            self.gt_his.append(gt_pose)

            print('pose = ({:.2f}, {:.2f}, {:.2f})'.format(mypose[0], mypose[1], math.degrees(mypose[2]))) 
            #print('ground truth pose = ({:.2f}, {:.2f}, {:.2f})'.format(gt_pose[0], gt_pose[1], math.degrees(gt_pose[2])))
        
        if round(mypose[0], 1) == round(self.goal[0], 1) and round(mypose[1], 1) == round(self.goal[1], 1) and round(math.degrees(mypose[2])) != 0:
            return False
        return True                                  # return False to exit the loop

    def run(self):
        """
        Run this instance of the Top Level loop
        debug level can be used to selectively print debug information
        """
        nstep = 0
        pars = self.robot_gwy.get_params()
        self.observer.init_pose(pars)
        self.gridmap = Gridmap(pars)
        self.gridmap.init_grid()
        self.controller.init_controls(goal=self.goal, init_pose=(2.0, -2.0))

        while self.step() and not ros.is_shutdown():
            time.sleep(self.tcycle)
            nstep = nstep+1

        self.gridmap.print_grid(layer = 'ukn')
        self.gridmap.print_grid(layer = 'ept')
        self.gridmap.print_grid(layer = 'occ')
        self.gridmap.close_grid()

        self.robot_gwy.shutdown_robot()

        # Plot history
        if self.debug > 0:

            plt.figure()

            # Get x and y values
            x_est = [est_pose[0] for est_pose in self.est_his]
            y_est = [est_pose[1] for est_pose in self.est_his]
            plt.plot(x_est, y_est, label="Estimated trajectory")

            x_gt = [gt_pose[0] for gt_pose in self.gt_his]
            y_gt = [gt_pose[1] for gt_pose in self.gt_his]
            plt.plot(x_gt, y_gt, label="True trajectory")

            plt.legend()

            plt.show()



        if self.debug > 1:
            f = '/home/alvaro/AIRob-2024/lessons_ws/occ.pgm'
            reader = Reader()
            image = reader.read_pgm(f)
            width = reader.width

            plt.figure()
            plt.imshow(image, cmap='gray')
            plt.axis('off')  # Hide axes
            plt.show()

            f = '/home/alvaro/AIRob-2024/lessons_ws/ept.pgm'
            reader = Reader()
            image = reader.read_pgm(f)
            width = reader.width

            plt.figure()
            plt.imshow(image, cmap='gray')
            plt.axis('off')  # Hide axes
            plt.show()