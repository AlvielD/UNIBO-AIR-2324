
from fcontrol import Behavior
from fcontrol import ramp_up, ramp_down, triangle
from fcontrol import global_to_local, local_to_global
from math import atan2, degrees, sqrt

OBJECT_POS = {
    'bed1': (-8.1, 7.7),
    'wardrobe': (-8.0, 5.8),
    'stove': (-6.5, -2.3),
    'table2': (5.0, 0.0),
    'table3': (5.0, 5.0),
    'D1': (1.1, -5.5, 1.57),
    'D2': (1.1, 2.5, 1.57),
    'D3': (-2.5, -1.5, 0),
    'D4': (-2.9, -5.5, 1.57)
}

class GoToTarget (Behavior):
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
        self.target = [target[0], target[1], 0.0]   # target point in global coordinates
        self.tlocal = [0, 0, 0]                     # target point in robot's coordinates
    
    def update_state(self, state):
        """
        Update the relevant local state variables (self.state) at every control cycle
        Uses the 'mypose' estimate, passed as part of the state by the top-level loop
        Set the local state variables 'phi' and 'rho'
        """
        mypose = state['mypose']
        global_to_local(self.target, mypose, self.tlocal)
        xt = self.tlocal[0]
        yt = self.tlocal[1]
        self.state['phi'] = degrees(atan2(yt, xt))
        self.state['rho'] = sqrt(xt * xt + yt * yt)

    def setup(self):
        """
        Definition of the rules of our 'go to target' fuzzy controller
        Fuzzy interpretation is built from the local state variables 'phi' and 'rho'
        """

        
        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'TargetLeft'  : (ramp_up(10.0, 45.0), 'phi'),
            'TargetRight' : (ramp_down(-45.0, -10.0), 'phi'),
            'TargetAhead' : (triangle(-60.0, 0.0, 60.0), 'phi'),
            'TargetHere'  : (ramp_down(0.0, 2.0), 'rho')
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.5, 'Slow':0.1, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':40, 'MLeft':10, 'None':0, 'MRight':-10, 'Right':-40}, 'Vrot')
        }

        self.frules = {
            # Lastly, definition of the actual fuzzy rules
            'ToLeft'  : ("TargetLeft AND NOT(TargetHere)", 'Turn', 'Left'),
            'CLeft'   : ("TargetLeft AND TargetHere", "Turn", "MLeft"),
            'ToRight' : ("TargetRight AND NOT(TargetHere)", 'Turn', 'Right'),
            'CRight'  : ("TargetRight AND TargetHere", 'Turn', 'MRight'),
            'Far'     : ("TargetAhead AND NOT(TargetHere) AND NOT(TargetLeft) AND NOT(TargetRight)", 'Move', 'Fast'),
            'Stop'    : ("TargetHere", 'Move', 'None')
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
    def __init__(self, target = (0.0, 0.0)):
        super().__init__()
    

    def update_state(self, state):

        """
        Update the relevant local state variables (self.state) at every control cycle
        Uses the 'mypose' estimate, passed as part of the state by the top-level loop
        Set the local state variables 'phi' and 'rho'
        """
        
        # Recover sonar data from the robot's state
        sdata = state['sdata']

        # Take the front sonars of the left side of the robot
        lSonar1 = sdata[1][1]
        lSonar2 = sdata[2][1]

        # Take the front sonars of the right side of the robot
        rSonar1 = sdata[11][1]
        rSonar2 = sdata[10][1]

        # Take minimum value of sonars on each side
        minRSonar = min(rSonar1, rSonar2)
        minLSonar = min(lSonar1, lSonar2)

        # Difference in calibration
        # If calibration > 0 => uncalibrated to the left, we have to turn right
        # If calibration < 0 => uncalibrated to the right, we have to turn left
        self.state['calibration'] = minRSonar - minLSonar
        

    def setup(self):
        """
        Definition of the rules of our 'go to target' fuzzy controller
        Fuzzy interpretation is built from the local state variables 'phi' and 'rho'
        """

        
        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'CenterLeft'  : (ramp_down(-1.0, -0.2), 'calibration'),
            'CenterRight' : (ramp_up(0.2, 1.0), 'calibration'),
            'Centered' : (triangle(-0.3, 0.0, 0.3), 'calibration'),
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.3, 'Slow':0.1, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':30, 'MLeft':10, 'None':0, 'MRight':-10, 'Right':-30}, 'Vrot')
        }

        self.frules = {
            # Lastly, definition of the actual fuzzy rules
            'Go'        : ("Centered", 'Move', 'Slow'),
            'TurnLeft'  : ("CenterLeft", 'Turn', 'MLeft'),
            'TurnRight' : ("CenterRight", 'Turn', 'MRight'),
        }

        # The degree of achievement is given by the predicate 'TargetHere'
        self.fgoal = "TargetHere"

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()


class Controller():
    def __init__(self, goal=(0, 0), init_pose=(0.0), debug=0):
        """
        Initialize the controller, setting the global goal position
        Return True if the inizialization was successful
        """
        self.GOAL_POSITION = goal
        self.INIT_POSITION = init_pose

        self.vlin = 0.0
        self.vrot = 0.0

        self.achieved = 1.0

        self.behavior = None    # which behavior we are currently executing
        self.debug = debug
    

    def init_controls(self, goal, init_pose):
        self.GOAL_POSITION = goal
        self.INIT_POSITION = init_pose

    
    def set_behavior(self, operator):
        """
        Associate operators and behaviors
        """
        if operator[0] == 'GoTo':
            target_pos = OBJECT_POS[operator[1]]
            self.behavior = GoToTarget(target_pos)
        elif operator[0] == 'Cross':
            target_pos = OBJECT_POS[operator[1]]
            self.behavior = Cross()


    def get_achieved(self):
        return self.achieved


    def compute_ctr (self, state):
        """
        Action decision. Compute control values (vlin, vrot) given the current robot's pose
        Returns the control values (vlin, vrot), as a pair
        """

        if self.behavior != None:
            self.achieved = self.behavior.run(state, self.debug)
            self.vlin = self.behavior.get_vlin()
            self.vrot = self.behavior.get_vrot()
            if self.debug > 0:
                print('Goal achievement: {:.2f}'.format(self.achieved))
            if self.debug > 0:
                print('(vlin, vrot)) = ({:.2f}, {:.2f})'.format(self.vlin, degrees(self.vrot))) 

        return (self.vlin, self.vrot, self.achieved)