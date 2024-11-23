"""
============== UniBo: AI and Robotics 2024 ==============
Base code: gateway to the robot (or simulator)
This is a dummy version, most values and funtions are place-holders
It needs to be customized by the students for the different labs

(c) 2024 Alessandro Saffiotti
"""
import rospy as ros
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math


class RobotGateway():
    def __init__(self):
        """
        This should set up the communication channels with the robot (or simulator)
        and perform any initialization needed
        """

        self.parameters = {                  # these are for the Tiago robot
            'wheel_radius' : 0.0985,
            'wheel_axis' : 0.4044
        }

        # Subscribe to the topic in charge of robot joint states
        self.wheels_sub = ros.Subscriber("/joint_states", JointState, self.wheels_callback)
        self.gt_odom_sub = ros.Subscriber("/ground_truth_odom", Odometry, self.ground_truth_callback)
        self.vel_pub = ros.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)

        # Wheel distance
        self.dl = 0.0
        self.dr = 0.0

        # Ground truth position of the robot
        self.gt_x = 3.0
        self.gt_y = 0.0
        self.gt_th = 0.0

        # Mobile base velocity
        self.vlin = 0.0
        self.vrot = 0.0

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