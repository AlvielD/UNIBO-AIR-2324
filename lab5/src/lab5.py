#!/usr/bin/env python3
import toplevel, pyhop, htn_domain
import rospy as ros

if __name__ == '__main__':
    ros.init_node('master')

    goal1 = [('transport', 'box3', 'bed1')]
    goal2 = [('navigate_to', 'wardrobe1')]
    goal3 = [('transport', 'box3', 'wardrobe1')]
    goal4 = [('transport', 'box1', 'stove1')]
    goal5 = [('navigate_to', 'box2')]

    goal6 = [('transport', 'box1', 'stove1')]
    goal7 = [('transport', 'box3', 'table3')]
    goal8 = [('navigate_to', 'table2')]
    goal9 = [('navigate_to', 'wardrobe1')]

    toplevel = toplevel.TopLevelLoop(tcycle = 0.1, debug = 2, random_boxes=False, close_doors=False)
    toplevel.run(maxsteps = 0, goal = goal1)