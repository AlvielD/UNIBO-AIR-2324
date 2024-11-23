#!/usr/bin/env python3
import toplevel, pyhop, htn_domain
import rospy


if __name__ == '__main__':
    rospy.init_node('master')
    toplevel = toplevel.TopLevelLoop(goal = (8.0, -4.0), tcycle = 0.1, debug = 1)

    # Define and solve some planning problems

    #pyhop.print_operators()
    #pyhop.print_methods()

    state1 = htn_domain.State()

    state1.room['bed1'] = 'Room1'
    state1.room['wardrobe'] = 'Room1'
    state1.room['fridge'] = 'Room2'
    state1.room['stove'] = 'Room2'
    state1.room['sink'] = 'Room2'
    state1.room['table1'] = 'Room2'
    state1.room['table2'] = 'Room4'
    state1.room['table3'] = 'Room4'

    state1.room['box1'] = 'Room1'
    state1.room['box2'] = 'Room2'
    state1.room['box3'] = 'Room3'
    state1.room['box4'] = 'Room4'

    state1.connects['D1'] = ('Room3', 'Room4')
    state1.connects['D2'] = ('Room1', 'Room4')
    state1.connects['D3'] = ('Room1', 'Room3')
    state1.connects['D4'] = ('Room2', 'Room3')

    state1.doors['D1'] = 'Open'
    state1.doors['D2'] = 'Open'
    state1.doors['D3'] = 'Open'
    state1.doors['D4'] = 'Open'

    state1.pos['me'] = 'table2'
    state1.room['me'] = 'Room4'
    state1.holding['me'] = None

    #state1.pos['me'] = 'fridge'
    #state1.room['me'] = 'Room2'

    print("Initial state:")
    pyhop.print_state(state1)

    #myplan = pyhop.pyhop(state1, [('navigate_to', 'table2')], verbose=2)
    #myplan = pyhop.pyhop(state1, [('navigate_to', 'table3')], verbose=2)
    #myplan = pyhop.pyhop(state1, [('navigate_to', 'bed1')], verbose=3)
    myplan = pyhop.pyhop(state1, [('navigate_to', 'stove')], verbose=3)
    #myplan = pyhop.pyhop(state1, [('navigate_to', 'wardrobe')], verbose=2)
    #myplan = pyhop.pyhop(state1, [('fetch', 'box1')], verbose=3)
    #myplan = pyhop.pyhop(state1, [('transport', 'box1', 'stove')], verbose=3)

    print('')
    print("Plan:", myplan)

    toplevel.run(maxsteps = 0, plan = myplan)