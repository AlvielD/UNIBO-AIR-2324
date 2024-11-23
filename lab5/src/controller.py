
import robot_map
import rospy as ros
import math
from environment_pkg.srv import BoxUpdatePos, BoxPickUp, BoxPutDown, DoorOpen, DoorStatus, BoxPos
from fcontrol import Behavior
from fcontrol import ramp_up, ramp_down, triangle, trapezoid
from fcontrol import global_to_local, local_to_global
from math import atan2, degrees, sqrt


class GoTo(Behavior):
    """
    Instance of the Behavior class, which defines a fuzzy controller
    to navigate to a given target point target = (x,y)
    The setup function is called once when the behavior is created,
    and it creates all the fuzzy predicates, actions and rules that
    define the behavior's control strategy.
    The update_state function is called at every cycle and it sets
    the internal variables based on the passed robot's state
    The run function is also called at every cycle, and it runs
    the fuzzy controller with the behavior's rules and sets the
    output control variables vlin, vrot: this function is inherited
    from the FControl class (superclass of Behavior)
    The values of vlin,vrot are fetched via the get_vlin and get_vrot
    functions, which are inherited from the Behavior class
    """
    def __init__(self, target = (0.0, 0.0)):
        super().__init__()
        self.target = [target[0], target[1], target[2], target[3]]   # target point in global coordinates
        self.tlocal = [0, 0, 0]                     # target point in robot's coordinates
    
    def update_state(self, state):
        """
        Update the relevant local state variables (self.state) at every control cycle
        Uses the 'mypose' estimate, passed as part of the state by the top-level loop
        Set the local state variables 'phi' and 'rho'
        """
        mypose = state['mypose']
        sdata = state['sdata']

        global_to_local(self.target, mypose, self.tlocal)

        xt = self.tlocal[0]
        yt = self.tlocal[1]

        self.state['phi'] = degrees(atan2(yt, xt))
        self.state['rho'] = sqrt(xt * xt + yt * yt) - self.target[3]
        self.state['frontSonar'] = sdata[0][1]
        self.state['leftSonar30'] = sdata[1][1]
        self.state['leftSonar60'] = sdata[2][1]
        self.state['rightSonar30'] = sdata[11][1]
        self.state['rightSonar60'] = sdata[10][1]

    def setup(self):
        """
        Definition of the rules of our 'go to target' fuzzy controller
        Fuzzy interpretation is built from the local state variables 'phi' and 'rho'
        """

        
        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'TargetLeft'      : (ramp_up(0.0, 45.0), 'phi'),
            'TargetRight'     : (ramp_down(-45.0, 0.0), 'phi'),
            'TargetAhead'     : (triangle(-30.0, 0.0, 30.0), 'phi'),
            'TargetHere'      : (ramp_down(0.0, 2.0), 'rho'),
            'ObstacleLeft30'  : (ramp_down(1.0, 2.0), 'leftSonar30'),
            'ObstacleRight30' : (ramp_down(1.0, 2.0), 'rightSonar30'),
            'ObstacleLeft60'  : (ramp_down(0.3, 1.3), 'leftSonar60'),
            'ObstacleRight60' : (ramp_down(0.3, 1.3), 'rightSonar60'),
            'ObstacleAhead'   : (ramp_down(1.0, 3.0), 'frontSonar'),
            'Crash'           : (ramp_down(0.5, 1.5), 'frontSonar'),
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.5, 'Slow':0.2, 'None':0, 'Back':-0.2}, 'Vlin'),
            'Turn' : ({'Left':40, 'MLeft':20, 'None':0, 'MRight':-20, 'Right':-40}, 'Vrot')
        }

        self.frules = {
            # Lastly, definition of the actual fuzzy rules
            # Basic GoTo rules
            'ToLeft'        : ("TargetLeft AND NOT(ObstacleRight30 OR ObstacleRight60)", 'Turn', 'Left'),
            'ToRight'       : ("TargetRight AND NOT(ObstacleLeft30 OR ObstacleLeft60)", 'Turn', 'Right'),
            'Far'           : ("TargetAhead", 'Move', 'Fast'),
            # Careful rules (slightly avoid obstacles)
            'CLeft'         : ("TargetRight AND (ObstacleRight30 OR ObstacleRight60)", "Turn", "MLeft"),
            'CRight'        : ("TargetLeft AND (ObstacleLeft30 OR ObstacleLeft60)", 'Turn', 'MRight'),
            #'Slow'          : ("ObstacleLeft30 OR ObstacleLeft60 OR ObstacleRight30 OR ObstacleRight60 OR ObstacleAhead", 'Move', 'Slow'),
            # Avoid rules
            'AvoidLeft'     : ("(ObstacleRight30 OR ObstacleRight60) AND NOT(TargetRight)", 'Turn', 'Left'),
            'AvoidRight'    : ("(ObstacleLeft30 OR ObstacleLeft60) AND NOT(TargetLeft)", 'Turn', 'Right'),
            'Stop'          : ("TargetHere", 'Move', 'Slow'),
            'Back'          : ("NOT(TargetHere) AND Crash", 'Move', 'Back'),
        }

        # The degree of achievement is given by the predicate 'TargetHere'
        self.fgoal = "TargetHere"

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()



class Avoid(Behavior):
    def __init__(self, target = (0.0, 0.0)):
        super().__init__()
        self.target = [target[0], target[1], 0.0]   # target point in global coordinates
        self.tlocal = [0, 0, 0]                     # target point in robot's coordinates
    
    def update_state(self, state):
        """
        Update the relevant local state variables (self.state) at every control cycle
        Uses the 'mypose' estimate, passed as part of the state by the top-level loop
        Set the local state variables 'phi' and 'rho'
        """
        mypose = state['mypose']
        sdata = state['sdata']

        global_to_local(self.target, mypose, self.tlocal)
        xt = self.tlocal[0]
        yt = self.tlocal[1]
        
        self.state['phi'] = degrees(atan2(yt, xt))
        self.state['rho'] = sqrt(xt * xt + yt * yt)

        self.state['frontSonar'] = sdata[0][1]
        self.state['leftSonar'] = sdata[1][1]
        self.state['backSonar'] = sdata[6][1]
        self.state['rightSonar'] = sdata[11][1]

    def setup(self):
        """
        Definition of the rules of our 'avoid' fuzzy controller
        Fuzzy interpretation is built from the local state variables 'phi' and 'rho'
        """

        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'ObstacleLeft'  : (ramp_down(0.0, 3.0), 'leftSonar'),
            'ObstacleRight' : (ramp_down(0.0, 3.0), 'rightSonar'),
            'ObstacleAhead'  : (ramp_down(0.0, 3.0), 'frontSonar'),
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.3, 'Slow':0.1, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':30, 'MLeft':10, 'None':0, 'MRight':-10, 'Right':-30}, 'Vrot')
        }

        self.frules = {
            # Lastly, definition of the actual fuzzy rules
            'ToLeft'  : ("ObstacleRight", 'Turn', 'Left'),
            'ToRight' : ("ObstacleLeft", 'Turn', 'Right'),
            'Go'      : ("NOT(ObstacleAhead)", 'Move', 'Fast'),
            'Back'    : ("ObstacleAhead AND ObstacleRight AND ObstacleLeft", 'Move', 'Back')
        }

        # The degree of achievement is given by the predicate 'TargetHere'
        self.fgoal = "NOT(ObstacleAhead) AND NOT(ObstacleRight) AND NOT(ObstacleLeft)"

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()



class FollowObject(Behavior):
    def __init__(self, target = (0.0, 0.0)):
        super().__init__()
        self.target = [target[0], target[1], 0.0]   # target point in global coordinates
        self.tlocal = [0, 0, 0]                     # target point in robot's coordinates
    
    def update_state(self, state):
        """
        Update the relevant local state variables (self.state) at every control cycle
        Uses the 'mypose' estimate, passed as part of the state by the top-level loop
        Set the local state variables 'phi' and 'rho'
        """
        mypose = state['mypose']
        sdata = state['sdata']

        global_to_local(self.target, mypose, self.tlocal)
        xt = self.tlocal[0]
        yt = self.tlocal[1]
        
        self.state['phi'] = degrees(atan2(yt, xt))
        self.state['rho'] = sqrt(xt * xt + yt * yt)

        self.state['frontSonar'] = sdata[0][1]
        self.state['frontRSonar'] = sdata[11][1]
        self.state['frontLSonar'] = sdata[1][1]

    def setup(self):
        """
        Definition of the rules of our 'avoid' fuzzy controller
        Fuzzy interpretation is built from the local state variables 'phi' and 'rho'
        """

        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'ObjectAhead'  : (triangle(1.0, 2.0, 3.0), 'frontSonar'),
            'ObjectClose'  : (ramp_down(0.2, 2.0), 'frontSonar'),
            'ObjectCloseR'  : (ramp_down(0.2, 2.0), 'frontRSonar'),
            'ObjectCloseL'  : (ramp_down(0.2, 2.0), 'frontLSonar'),
            'ObjectLeft'   : (triangle(1.0, 2.0, 3.0), 'frontLSonar'),
            'ObjectRight'  : (triangle(1.0, 2.0, 3.0), 'frontRSonar')
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.3, 'Slow':0.1, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':30, 'MLeft':10, 'None':0, 'MRight':-10, 'Right':-30}, 'Vrot')
        }

        self.frules = {
            # Lastly, definition of the actual fuzzy rules
            'ToLeft'  : ("ObjectLeft", 'Turn', 'Left'),
            'ToRight' : ("ObjectRight", 'Turn', 'Right'),
            'Go'      : ("ObjectAhead", 'Move', 'Fast'),
            'Stop'    : ("ObjectClose", 'Move', 'None'),
            'StopR'    : ("ObjectCloseR", 'Move', 'None'),
            'StopL'    : ("ObjectCloseL", 'Move', 'None')
        }

        # The degree of achievement is given by the predicate 'TargetHere'
        self.fgoal = "ObjectClose"

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()

class Cross(Behavior):
    def __init__(self, target = (0.0, 0.0, 0.0)):
        super().__init__()
        self.state['steps'] = 0
        self.target = [target[0], target[1], target[2]]
        self.sym_target = robot_map.map.find_location(self.target)
        self.tlocal = [0.0, 0.0, 0.0]
        self.frontSonar = 3.0

    def update_state(self, state):

        """
        Update the relevant local state variables (self.state) at every control cycle
        Uses the 'mypose' estimate, passed as part of the state by the top-level loop
        Set the local state variables 'phi' and 'rho'
        """

        mypose = state['mypose']
        sdata = state['sdata']
        self.state['steps'] = self.state['steps'] + 1

        global_to_local(self.target, mypose, self.tlocal)

        # TODO: CHECK THIS APPROACH

        if self.target[2] == 0:
            orientation = math.sin(mypose[2]) if (self.target[1] - mypose[1]) > 0 else -math.sin(mypose[2])
            self.state['orientation'] = orientation
        elif self.target[2] == 1.57:
            orientation = math.cos(mypose[2]) if (self.target[0] - mypose[0]) > 0 else -math.cos(mypose[2])
            self.state['orientation'] = orientation
            

        # --------------------------

        self.state['angle'] = self.tlocal[2]

        # Take the frontSonar
        self.frontSonar = sdata[0][1]


        self.state['doorDist'] = min(sdata[5][1], sdata[7][1])
        # Difference in calibration
        # If calibration > 0 => uncalibrated to the left, we have to turn right
        # If calibration < 0 => uncalibrated to the right, we have to turn left
        self.state['calibration'] = sum([sonar[1] for sonar in sdata[9:]]) - sum([sonar[1] for sonar in sdata[1:4]])
        

    def setup(self):
        """
        Definition of the rules of our 'go to target' fuzzy controller
        Fuzzy interpretation is built from the local state variables 'phi' and 'rho'
        """

        
        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'Crossed'    : (ramp_up(0, 50), 'steps'),
            'CenterLeft'  : (ramp_down(-1.0, -0.2), 'calibration'),
            'CenterRight' : (ramp_up(0.2, 1.0), 'calibration'),
            'Centered' : (triangle(-0.4, 0.0, 0.4), 'calibration'),
            'Center': (ramp_up(0.0, 1.0), 'orientation'),
            'DoorDist' : (trapezoid(0.2, 0.5, 1.2, 3.0), 'doorDist')
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.25, 'Slow':0.15, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':30, 'MLeft':10, 'None':0, 'MRight':-10, 'Right':-30}, 'Vrot')
        }

        self.frules = {
            # Lastly, definition of the actual fuzzy rules
            'Go'        : ("Centered", 'Move', 'Fast'),
            'Slow'      : ("NOT(CenterLeft) AND NOT(CenterRight)", 'Move', 'Slow'),
            'TurnLeft'  : ("CenterLeft", 'Turn', 'MLeft'),
            'TurnRight' : ("CenterRight", 'Turn', 'MRight'),
            'Stop'      : ('DoorDist AND Centered', 'Move', 'None')
        }

        # The degree of achievement is given by the predicate 'TargetHere'
        self.fgoal = "DoorDist"

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()

    
    def run(self, state, debug=0):
        success, achievement = super().run(state, debug)
        
        #DOORS FAILURE
        
        if self.frontSonar < 0.5:
            if debug == 0: print("Door is actually closed. Updating map.")
            success = False # Make the action fail
            robot_map.map.properties[self.sym_target] = 'Closed'    # Update map
        
        
        return success, achievement


class Open(Behavior):
    def __init__(self, target):
        super().__init__()

        # Create the service to open the door
        target_str = f'{target[0]}oor{target[1]}'

        ros.wait_for_service(f'{target_str}/update_pos')
        self.door_srv = ros.ServiceProxy(f'{target_str}/update_pos', DoorOpen)


    def run(self, state, debug):
        success = self.door_srv(True).done
        achievement = 1 if success else 0

        return success, achievement
    

class Close(Behavior):
    def __init__(self, target):
        super().__init__()

        # Create the service to open the door
        target_str = f'{target[0]}oor{target[1]}'

        ros.wait_for_service(f'{target_str}/update_pos')
        self.door_srv = ros.ServiceProxy(f'{target_str}/update_pos', DoorOpen)


    def run(self, state, debug):
        success = self.door_srv(False).done
        achievement = 1.0 if success else 0.0

        return success, achievement


class PickUp(Behavior):
    def __init__(self, target):
        super().__init__()
        
        self.sym_target = target

        # Create service to pick up the box
        target_str = str(target).upper()
        ros.wait_for_service(f'{target_str}/pick_up')
        self.box_srv = ros.ServiceProxy(f'{target_str}/pick_up', BoxPickUp)
        self.box_dist = 3.0

    
    def update_state(self, state):
        sdata = state['sdata']
        self.box_dist = min(sdata[0][1], sdata[11][1], sdata[1][1])

    
    def run(self, state, debug):
        self.update_state(state)

        #success = self.box_srv(True).success    # Detection by service
        success = self.box_dist <= 0.6        # Detection by sensors
        if success: success = self.box_srv(True).success
        achievement = 0.0
        if success:
            achievement = 1.0
            robot_map.map.holding['me'] = self.sym_target

        return success, achievement


class PutDown(Behavior):
    def __init__(self, target):
        super().__init__()
        
        self.sym_target = target

        # Create service to pick up the box
        target_str = str(target).upper()
        ros.wait_for_service(f'{target_str}/put_down')
        self.box_srv = ros.ServiceProxy(f'{target_str}/put_down', BoxPutDown)

    
    def run(self, state, debug):

        achievement = 0.0
        success = self.box_srv(True).success
        if success:
            achievement = 1.0

        return success, achievement


class Controller():
    def __init__(self):
        """
        Initialize the controller, setting the global goal position
        Return True if the inizialization was successful
        """
        self.behavior = None        # top-level fuzzy behavior run by the controller
        self.achieved = 0.0         # level of achievement of current behavior
        self.vlin = 0.0             # current value for vlin control variable
        self.vrot = 0.0             # current value for vrot control variable

    
    def set_behavior(self, bname = None, bparam = None):
        """
        Initialize the controller, setting the behavior to be executed
        Return True if the inizialization is successful
        """
        if bname:
            self.behavior = globals()[bname](bparam)
        return True


    def get_achieved(self):
        return self.achieved


    def run(self, state, debug):
        """
        Action decision. Compute control values (vlin, vrot) given the current robot's pose
        Returns the control values (vlin, vrot), as a pair
        """

        if self.behavior != None:
            success, self.achieved = self.behavior.run(state, debug)
            self.vlin = self.behavior.get_vlin()
            self.vrot = self.behavior.get_vrot()
            if debug > 1:
                print('Goal achievement: {:.2f}'.format(self.achieved))
                print('(vlin, vrot)) = ({:.2f}, {:.2f})'.format(self.vlin, degrees(self.vrot))) 

        return success, self.achieved
    

    def get_vlin(self):
        return self.vlin
    

    def get_vrot(self):
        return self.vrot