"""
============== UniBo: AI and Robotics 2024 ==============
Base code: HTN planning domain for Pyhop planner
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
import pyhop
import math

class State(pyhop.State):
    def __init__(self):
        self.__name__ = "s1"
        self.pos = {}           # position of robot or objet: a symbolic name
        self.room = {}          # room of robot or objet
        self.door = {}          # doors' status: closed or open
        self.connects = {}      # doors' connectivity: pair of rooms
        self.holding = {}


###############################################################################
# OPERATORS
# First argument is current state, others are the operator's parameters.
###############################################################################

def GoTo (state, target):
    state.pos['me'] = target
    return state

def Cross (state, door):
    if (state.pos['me'] != door):
        return False
    if (state.room['me'] == state.connects[door][0]):
        state.room['me'] = state.connects[door][1]

        # If the robot is holding something, change also the location of its item
        if state.holding['me'] != None:
            state.room[state.holding['me']] = state.connects[door][1]
            
        return state
    if (state.room['me'] == state.connects[door][1]):
        state.room['me'] = state.connects[door][0]

        # If the robot is holding something, change also the location of its item
        if state.holding['me'] != None:
            state.room[state.holding['me']] = state.connects[door][0]

        return state
    return False

def Open(state, door):
    if(state.pos['me'] != door or state.door[door] == 'Open'):
        return False
    else:
        state.door[door] = 'Open'
        return state

def Close(state, door):
    if(state.pos['me'] != door or state.door[door] == 'Closed'):
        return False
    else:
        state.door[door] = 'Closed'
        return state
    
def PickUp(state, box):
    if state.holding['me'] != None:
        return False
    else:
        state.holding['me'] = box
        return state
    
def PutDown(state, box):
    if state.holding['me'] == None:
        return False
    else:
        state.holding['me'] = None
        return state



pyhop.declare_operators(GoTo, Cross, Open, Close, PickUp, PutDown)


###############################################################################
# METHODS
# First argument is current state, others are the method's parameters.
# They may call other methods, or executable operators.
###############################################################################

# Method to navigate when we are already at the target

def move_in_place (state, target):
    if state.pos['me'] == target:
        return []
    else:
        return False

# Method to navigate when the target is in the same room

def move_in_room(state, target):
    if state.room['me'] == state.room[target]:
        return [('GoTo', target)]
    else:
        return False

# Helper function to find connecting doors

def doors_between (state, room1, room2):
    doors = []
    for d in state.connects:
        if (state.connects[d][0] == room1 and state.connects[d][1] == room2) or (state.connects[d][0] == room2 and state.connects[d][1] == room1) :
            doors.append(d)
    return doors

# Helper function to find all the adjacent rooms to the target one

def rooms_between(state, target_room):
    rooms = []
    for d in state.connects:
        if state.connects[d][0] == target_room:
            rooms.append(state.connects[d][1])
        elif state.connects[d][1] == target_room:
            rooms.append(state.connects[d][0])
    return rooms

# Method to navigate when the target is in an adjacent room

"""
#GIVEN IMPLEMENTATION OF MOVE_ACROSS_ROOMS
#This version only allows the robot to move to adjacent rooms

def move_across_rooms(state, target):
    room1 = state.room['me']
    room2 = state.room[target]
    if room1 == room2:
        return False
    doors = doors_between(state, room1, room2)
    if doors == []:
        return False
    door = doors[0]
    if state.door[door] == 'Closed':
        return [('GoTo', door), ('Open', door), ('Cross', door), ('Close', door), ('GoTo', target)]
    else:
        return [('GoTo', door), ('Cross', door), ('GoTo', target)]
"""

# Generalization of move_across_rooms for any starting point and target
    
"""
FIRST IMPLEMENTATION OF THE MOVE_ACROSS_ROOMS USING BFS
This version creates the plan in a hardcode way, it should not be done like this

def move_across_rooms_bfs(state, target):
    start = state.room['me']
    end = state.room[target]

    # Initialize variables for the search
    queue = []
    visited = set()
    queue.append([start])

    # If the room is the same, we do not need to cross rooms.
    if start == end:
        return False

    # Apply the BFS algorithm
    while queue != []:
        path = queue.pop(0)
        node = path[-1]
        
        # If the node is the solution stop
        if node == end:
            break
        
        if node not in visited:
            visited.add(node)                       # Add the node to the visited list
            neighbors = rooms_between(state, node)  # Expand node

            # Set parents for each extended node
            for room in neighbors:
                new_path = list(path)
                new_path.append(room)
                queue.append(new_path)

    # Create the plan from the retrieved path of rooms.
    plan = []
    for i in range(len(path)-1):
        doors = doors_between(state, path[i], path[i+1])
        door = doors[0]
        if state.door[door] == 'Closed':
            plan.extend([('GoTo', door), ('Open', door), ('Cross', door), ('Close', door)])
        else: 
            plan.extend([('GoTo', door), ('Cross', door)])
    plan.append(('GoTo', target))

    return plan
"""

# Method to move from one room to another

def move_across_rooms(state, target):

    if state.pos['me'] == target:
        return False

    # Find the path to the target using our state
    path = bfs(state, target)

    # If there is no way to get to the target, False
    if path == []:
        return False

    # Find the door connecting to next room
    doors = doors_between(state, path[0], path[1])
    door = doors[0]

    if state.pos['me'] == door:
        return False

    return [('GoTo', door), ('cross_door', door), ('navigate_to', target)]


def cross_in_place(state, target):

    if state.pos['me'] == target:
        return False

    # Find the path to the target using our state
    path = bfs(state, target)

    # If there is no way to get to the target, False
    if path == []:
        return False

    # Find the door connecting to next room
    doors = doors_between(state, path[0], path[1])
    door = doors[0]

    if state.pos['me'] != door:
        return False

    return [('cross_door', door), ('navigate_to', target)]


# Helper function implementing the Breath First Search

def bfs(state, target):
    # Define start and end nodes
    start = state.room['me']
    end = state.room[target]

    # Initialize variables for the search
    queue = []
    visited = set()
    queue.append([start])

    # If start and end is the same return an empty path.
    if start == end:
        return []

    # Apply the BFS algorithm
    while queue != []:
        path = queue.pop(0)
        node = path[-1]
        
        # If the node is the solution stop
        if node == end:
            break
        
        if node not in visited:
            visited.add(node)                       # Add the node to the visited list
            neighbors = rooms_between(state, node)  # Expand node

            # Set parents for each extended node
            for room in neighbors:
                new_path = list(path)
                new_path.append(room)
                queue.append(new_path)

    return path

# Method to cross a closed door

def cross_closed_door(state, door):
    pos = state.pos['me']

    if state.door[pos] == 'Open':
        return False
    return [('Open', pos), ('Cross', pos), ('Close', pos)]

# Method to cross an open door

def cross_open_door(state, door):
    if state.door[door] == 'Closed':
        return False
    return [('Cross', door)]

# Method to fetch a box

def fetch_box(state, box):
    if state.holding['me'] == None:
        return [('navigate_to', box), ('PickUp', box)]
    else:
        return False


def already_fetched(state, box):
    if state.holding['me'] == box:
        return []
    else:
        return False


def transport_box(state, box, target):
    return [('fetch', box), ('navigate_to', target), ('PutDown', box)]



pyhop.declare_methods('navigate_to',  move_in_place, move_in_room, move_across_rooms, cross_in_place)
pyhop.declare_methods('cross_door', cross_open_door, cross_closed_door)
pyhop.declare_methods('fetch', fetch_box, already_fetched)
pyhop.declare_methods('transport', transport_box)

