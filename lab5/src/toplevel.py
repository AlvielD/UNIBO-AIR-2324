"""
============== UniBo: AI and Robotics 2024 ==============
Base code: top level execution loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
# Basic libraries
import time, math
import json
import rospy as ros
import numpy as np
import matplotlib.pyplot as plt

# Planning
import pyhop
from htn_domain import State

# Robot modules
from robot_gwy import RobotGateway
from controller import Controller
from observer import Observer
import robot_map

# Other services
from environment_pkg.srv import BoxUpdatePos, DoorOpen

# Utils TODO: Remove if not used
from pgm_reader import Reader

class TopLevelLoop:
    """
    Top level execution loop
    """

    def __init__(self,
                 goal = [],             # top level task, passed to the HTN planner
                 mypose = None,         # robot's starting pose, if None, take it from the map
                 tcycle = 0.1 ,         # cycle time, in sec
                 debug = 0,             # (0: none, 1: only outer SPA loop, 2: inner control loop, 3: details)
                 random_boxes = False,  # Wether to change randomly the positions of the boxes
                 close_doors = False,   # Wether to close doors
                 ):
        self.goal = goal
        self.tcycle = tcycle
        self.debug = debug
        self.mypose = mypose            # estimated robot's pose
        self.pars  = {}                 # robot's parameters
        self.ctr = None                 # controller instance

        # MONITORING DATA
        # Position related
        self.est_his = []   
        self.gt_his = []
        # Proximity related
        self.minSonar_his = []
        # Actions related
        self.actions_list = ['GoTo', 'Cross', 'Open', 'Close', 'PickUp', 'PutDown']
        self.action_his = {action: {'ex_time': [], 'n_fail': 0, 'n_complete': 0} for action in self.actions_list}

        # Define robot modules
        self.robot_gwy = RobotGateway()
        self.observer = Observer(robot_map.map.startpose)
        self.controller = Controller()

        # Box positions service
        self.box1_update_pos = ros.ServiceProxy('BOX1/update_position', BoxUpdatePos)
        self.box2_update_pos = ros.ServiceProxy('BOX2/update_position', BoxUpdatePos)
        self.box3_update_pos = ros.ServiceProxy('BOX3/update_position', BoxUpdatePos)

        if random_boxes:
            self.activate_rdm_positioning_box()

        if close_doors:
            self.close_doors()



    def activate_rdm_positioning_box(self):
        self.box1_update_pos(15, 0.8, True)
        self.box2_update_pos(15, 0.4, True)
        self.box3_update_pos(15, 0.4, True)

    def close_doors(self):
        # Change here to close or open doors: False => Closed, True => Open (I know this should be done better but I had no time)
        door_srv = ros.ServiceProxy('Door1/update_pos', DoorOpen)
        door_srv(False)
        door_srv = ros.ServiceProxy('Door2/update_pos', DoorOpen)
        door_srv(False)
        door_srv = ros.ServiceProxy('Door3/update_pos', DoorOpen)
        door_srv(False)
        door_srv = ros.ServiceProxy('Door4/update_pos', DoorOpen)
        door_srv(False)

    def print_pose(self, mypose):
        print('pose = ({:.2f}, {:.2f}, {:.2f})\n'.format(mypose[0], mypose[1], math.degrees(mypose[2]))) 


    def sense_plan_act(self, goal, maxsteps = 0):
        """
        The outer "sense plan act" loop
        """
        # the 'S' part, fill the state defined in our domain with current data
        state = State()
        self.get_state(state)
        if self.debug > 0:
            print("Planner called from initial state:")
            pyhop.print_state(state)

        # the 'P' part, generate a plan for the given goal in the current state
        plan = pyhop.pyhop(state, goal, verbose=2)
        if self.debug > 0: print("Planner returns plan:", plan)
        result = None
        while plan:

            # the 'A' part, execute the plan action by action
            result = self.execute_plan(plan, maxsteps)       # A = Act
            if self.debug > 0:
                if result:
                    print("Plan execution completed!")
                    break
                else:
                    print("Plan execution failed! Replanning...")
                    self.get_state(state)                       # Update state
                    if self.debug > 0:
                        print("Planner called from state:")
                        pyhop.print_state(state)
                    plan = pyhop.pyhop(state, goal, verbose=2)  # Replan
                    if self.debug > 0: print("Planner returns plan:", plan)
        return result


    def get_state(self, state):
        """
        Fill the given state with the current values, taken from the map
        or from the robot's sensors
        """
        # first we set the static part of the state, taken from the map
        # see in the map which objects (doors) connect rooms to one another
        for room1 in robot_map.map.topology:
            for room2 in robot_map.map.topology:
                if room1 == room2:
                    continue
                for object1 in robot_map.map.topology[room1]:
                    for object2 in robot_map.map.topology[room2]:
                        if object1 == object2:
                            state.connects[object1] = (room1, room2)
        # set their status
        for door in state.connects:
            state.door[door] = robot_map.map.properties[door]
        # see what is the room of each object (except the doors above)
        for room, contents in robot_map.map.topology.items():
            for object in contents:
                if object in state.connects:
                    continue
                state.room[object] = room

        # second we set the dynamic part of the state, updated through the robot's (real or virtual) sensors

        # BOXES' STATE
        state.pos['box1'] = self.robot_gwy.get_box_position('box1')
        state.pos['box2'] = self.robot_gwy.get_box_position('box2')
        state.pos['box3'] = self.robot_gwy.get_box_position('box3')

        state.room['box1'] = robot_map.map.find_room(state.pos['box1'])
        state.room['box2'] = robot_map.map.find_room(state.pos['box2'])
        state.room['box3'] = robot_map.map.find_room(state.pos['box3'])

        # Update box positions on the map
        robot_map.map.locations.update({
            'box1': state.pos['box1'],
            'box2': state.pos['box2'],
            'box3': state.pos['box3']    
        })

        # ROBOT'S STATE
        state.room['me'] = robot_map.map.find_room(self.mypose)
        state.pos['me']  = robot_map.map.find_location(self.mypose)
        state.pos['me_coords'] = self.mypose
        state.holding['me'] = robot_map.map.holding['me']


    def execute_plan(self, plan, maxsteps):
        """
        The "Act" part of the SPA loop
        Pop each action in the plan in sequence, and execute it
        Return True for successful plan execution, False for failure
        """
        if self.debug > 0: print("Executing plan")
        for action in plan:
            result = self.execute_action(action)
            if result == False:
                break
        return result
    

    def execute_action(self, action, threshold = 0.9, maxsteps = 0):
        """
        The inner action execution loop
        Execute 'action' until its degree of achievement is greter than 'threshold'
        timeout with failure after 'maxsteps' steps (zero = no timeout)
        Return True for successful execution, False for failure
        """
        if self.debug > 0: print("Executing action:", action)
        nsteps = 0

        # set the behavior to be run by the controller
        behavior = action[0]
        if behavior in ['Open', 'Close', 'PickUp', 'PutDown']: # because these call a service using the door's name
            param = action[1]   
        else:
            param = robot_map.map.locations[action[1]]
        self.controller.set_behavior(bname = behavior, bparam = param)

        # run the main control pipeline until completion, or failure
        start = time.time() # Measure start time
        while True:
            if ros.is_shutdown():    # ROS was killed
               end = time.time()
               return False
            nsteps += 1
            if(maxsteps > 0 and nsteps < maxsteps):     # timeout
                if self.debug > 0: print("Max number of steps reached: exiting")
                end = time.time()
                self.action_his[behavior]['ex_time'].append(end-start)
                self.action_his[behavior]['n_fail'] += 1
                return False
            result, done = self.step()                  # execute control pipeline
            if result == False:                         # behavior failed
                end = time.time()
                self.action_his[behavior]['ex_time'].append(end-start)
                self.action_his[behavior]['n_fail'] += 1
                if self.debug > 0: print("Action ", action, " failed")
                return False                            # action failed
            if done > threshold:                        # behavior completed
                end = time.time()
                self.action_his[behavior]['ex_time'].append(end-start)
                self.action_his[behavior]['n_complete'] += 1
                if self.debug > 0: print("Action ", action, " completed")
                return True                             # action completed



    def step(self):
        """
        The basic control pipeline: read sensors, estimate state, decide controls, send controls
        Return True for successful execution, plus the current degree of achievement
        """

        wl, wr = self.robot_gwy.get_wheel_encoders()        # read proprioceptive sensors
        sdata = self.robot_gwy.get_sonar_data()             # read exteroceptive sensors (sonars)

        if self.debug > 1: print('sonars =', ' '.join(['{:.2f}'.format(s[1]) for s in sdata]))

        self.mypose = self.observer.update_pose(wl, wr)         # estimate state (robot's pose)
        gt_pose = self.robot_gwy.get_ground_truth_values()

        # Store estimated and ground truth positions (x, y)
        self.est_his.append(self.mypose)
        self.gt_his.append(gt_pose)

        # Store minimum sonar distance
        self.minSonar_his.append((min([sonar[1] for sonar in sdata]), time.time()))

        state = {'mypose': self.mypose, 'sdata': sdata}         # Build state to pass it to the controller

        success, achieved = self.controller.run(state, self.debug)    # decide action (robot's vel)
        vlin = self.controller.get_vlin()
        vrot = self.controller.get_vrot()
        
        self.robot_gwy.set_vel_values(vlin, vrot)          # send control action  
        if self.debug > 1: self.print_pose(self.mypose)

        time.sleep(self.tcycle)          
        return success, achieved


    def plot_trajectory(self, f_name):
        # Plot history
        plt.figure()

        # Get x and y values
        x_est = [est_pose[0] for est_pose in self.est_his]
        y_est = [est_pose[1] for est_pose in self.est_his]
        plt.plot(x_est, y_est, label="Estimated trajectory")

        x_gt = [gt_pose[0] for gt_pose in self.gt_his]
        y_gt = [gt_pose[1] for gt_pose in self.gt_his]
        plt.plot(x_gt, y_gt, label="True trajectory")

        plt.legend()
        plt.savefig(f'src/Lab5/data/optional2/trajectory_{f_name}.png')
        plt.show()

    
    def plot_minSonar(self, f_name):
        plt.figure()

        y = [sonar[0] for sonar in self.minSonar_his]   # Min sonar value
        x = [sonar[1] for sonar in self.minSonar_his]   # Time stamp

        plt.plot(x, y, label='Min distance from robot to obstacles')
        plt.savefig(f'src/Lab5/data/optional2/minSonar_{f_name}.png')
        plt.show()

    
    def plot_minSonarTraj(self, f_name):
        # Plot history
        plt.figure()

        x_gt = [gt_pose[0] for gt_pose in self.gt_his]
        y_gt = [gt_pose[1] for gt_pose in self.gt_his]
        c = [sonar[0] for sonar in self.minSonar_his]
        sc = plt.scatter(x_gt, y_gt, c=c, cmap='viridis', label="True trajectory")

        cbar = plt.colorbar(sc)
        cbar.set_label('Value')

        plt.legend()
        plt.savefig(f'src/Lab5/data/optional2/minSonarTraj_{f_name}.png')
        plt.show()


    def run(self, maxsteps = 0, goal = None):
        """
        Run this instance of the Top Level loop
        debug level can be used to selectively print debug information
        """
        if goal:
            self.goal = goal
        if self.mypose == None:
            self.mypose = robot_map.map.startpose
        
        # Init the observer with robot's parameters
        self.observer.init_pose(self.robot_gwy.get_params())
        self.sense_plan_act(self.goal, maxsteps)
        self.robot_gwy.shutdown_robot()

        # TEST INFORMATION
        # Update action history
        for behavior in self.actions_list:
            self.action_his[behavior].update({'ex_time': np.mean(self.action_his[behavior]['ex_time'])}) 

        print(self.action_his)

        f_name = goal[0]
        for arg in goal[1:]:
            f_name += f'_{arg}'
        
        with open(f'src/Lab5/data/optional2/{f_name}.json', 'w') as file:
            json.dump(self.action_his, file)

        self.plot_trajectory(f_name)
        self.plot_minSonar(f_name)
        self.plot_minSonarTraj(f_name)



    
