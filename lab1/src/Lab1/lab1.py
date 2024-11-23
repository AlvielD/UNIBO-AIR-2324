#!/usr/bin/env python3
import toplevel
import rospy as ros
from std_msgs.msg import String

if __name__ == '__main__':
    ros.init_node("top_level")  # Init the ROS node

    toplevel = toplevel.TopLevelLoop(goal = (4.0, 2.0), debug = 1)
    toplevel.run()