"""
============== UniBo: AI and Robotics 2024 ==============
Base code: gateway to the robot (or simulator)
This is a dummy version, most values and funtions are place-holders
It needs to be customized by the students for the different labs

(c) 2024 Alessandro Saffiotti
"""
import rospy as ros
from sensor_msgs.msg import JointState, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math
import re


class RobotGateway():
    def __init__(self):
        """
        This should set up the communication channels with the robot (or simulator)
        and perform any initialization needed
        """

        self.parameters = {                  # these are for the Tiago robot
            'wheel_radius' : 0.0985,
            'wheel_axis' : 0.4044,
            'sonar_num' : 12,
            'sonar_maxrange' : 2.0,
            'sonar_delta' : math.radians(25.0),
            'sonar_poses' : [self.sonar_ring_pose(0.0),
                            self.sonar_ring_pose(30.0),
                            self.sonar_ring_pose(60.0),
                            self.sonar_ring_pose(90.0),
                            self.sonar_ring_pose(120.0),
                            self.sonar_ring_pose(150.0),
                            self.sonar_ring_pose(180.0),
                            self.sonar_ring_pose(210.0),
                            self.sonar_ring_pose(240.0),
                            self.sonar_ring_pose(270.0),
                            self.sonar_ring_pose(300.0),
                            self.sonar_ring_pose(330.0)]
        }

        # Subscribe to the topic in charge of robot joint states
        self.wheels_sub = ros.Subscriber("/joint_states", JointState, self.wheels_callback)
        self.gt_odom_sub = ros.Subscriber("/ground_truth_odom", Odometry, self.ground_truth_callback)
        self.vel_pub = ros.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
    
        # Initialize and array containing sonar topic names i.e. [sonar01_base, sonar02_base, ..., sonarN_base]
        self.sonar_keys = []
        for i in range(1, self.parameters['sonar_num']+1):
            n_sonar = f"0{i}" if i < 10 else i
            self.sonar_keys.append(f"sonar{n_sonar}_base")

        # Array of sonar subscribers
        self.sonar_subs = [ros.Subscriber(f"/{sonar_key}", Range, self.sonar_callback) for sonar_key in self.sonar_keys]

        # Wheel distance
        self.dl = 0.0
        self.dr = 0.0

        # Ground truth position of the robot
        self.gt_x = 3.0
        self.gt_y = -2.0
        self.gt_th = 0.0

        # Mobile base velocity
        self.vlin = 0.0
        self.vrot = 0.0

        # Initialize dictionary of sonar poses
        self.sonar_ranges = {sonar_key: 0.0 for sonar_key in self.sonar_keys}

        print("Robot initialized")


    def shutdown_robot(self):
        """
        This should perform any finalization needed on the robot,
        and close the communication channels
        """
        print("Robot shut")

    
    def ground_truth_callback(self, req):
        self.gt_x = req.pose.pose.position.x
        self.gt_y = req.pose.pose.position.y

        # Recover orientation from ground truth topics (given in quaternions)
        orientation_qt = req.pose.pose.orientation

        # Convert quaternion to Euler angles
        (_, _, self.gt_th) = euler_from_quaternion([orientation_qt.x,
                                                    orientation_qt.y,
                                                    orientation_qt.z,
                                                    orientation_qt.w])

    
    def get_ground_truth_values(self):
        return (self.gt_x, self.gt_y, self.gt_th)


    def wheels_callback(self, req):
        self.dl = req.position[req.name.index('wheel_left_joint')]
        self.dr = req.position[req.name.index('wheel_right_joint')]


    def get_wheel_encoders(self):
        """
        Get current values of wheel encoders, which indicate the current position
        of each wheel in radiants
        """

        return (self.dl, self.dr)
    

    def get_params(self):
        return self.parameters


    def set_vel_values(self, vlin, vrot):
        """
        Set new linear and rotational velocities for the robot's base
        vlin is m/sec vrot is rad/sec
        Returns True if successful
        """
        # Build the geometry_msg/Twist using the new velocities
        msg = Twist()
        msg.linear.x = vlin     # Moves forward on robot's x axis
        msg.angular.z = vrot    # Rotates around itself

        try:
            self.vel_pub.publish(msg)
        except:
            return False

        return True
    

    def sonar_ring_pose(self, bearing):
        rho = 0.260
        phi = math.radians(bearing)
        x = rho * math.cos(phi)
        y = rho * math.sin(phi)
        return (x, y, phi)
    

    def sonar_callback(self, req):
        # Get data from the sonar
        sonar_id = req.header.frame_id
        sonar_range = req.range

        # Extract substring containing the id
        match = re.search(r'\d+', sonar_id)
        sonar_id = match.group()
        sonar_key = self.sonar_keys[int(sonar_id)-1]    # Get sonar key related to its id

        # Update the sonar data
        self.sonar_ranges[sonar_key] = sonar_range


    def get_sonar_data(self):
        """
        Get current values from the sonar ring
        Returns an array of readings in the form (sonar pose, range)
        Sonar pose is (x, y, th) in robot's base frame
        This dummy version returns range 0.0 for all sonars
        """
        res = []
        for i in range(self.parameters['sonar_num']):
            res.append((self.parameters['sonar_poses'][i], list(self.sonar_ranges.values())[i]))
        return res