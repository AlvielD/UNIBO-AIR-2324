
import math

class Controller():
    def __init__(self, goal=(0, 0), init_pose=(0.0)):
        """
        Initialize the controller, setting the global goal position
        Return True if the inizialization was successful
        """
        self.GOAL_POSITION = goal
        self.INIT_POSITION = init_pose

        self.current_vlin = 0.0
        self.current_vrot = 0.0
    

    def init_controls(self, goal, init_pose):
        self.GOAL_POSITION = goal
        self.INIT_POSITION = init_pose


    def compute_ctr (self, mypose):
        """
        Action decision. Compute control values (vlin, vrot) given the
        current robot's pose and the global GOAL POSITION
        Returns the control values (vlin, vrot), as a pair
        """

        # DUMMY STRATEGY
        if abs(mypose[0] - 2.0) < 0.2 and \
        abs(mypose[1] - (-2.0)) < 0.2 and \
        abs(math.degrees(mypose[2]) - 0) < 2.0:
            self.current_vlin = 0.0
            self.current_vrot = 0.1

        if abs(mypose[0] - 2.0) < 0.2 and \
        abs(mypose[1] - (-2.0)) < 0.2 and \
        abs(math.degrees(mypose[2]) - 90) < 2.0:
            self.current_vlin = 0.1
            self.current_vrot = 0.0

        if abs(mypose[0] - 2.0) < 0.2 and \
        abs(mypose[1] - 3.0) < 0.2 and \
        round(math.degrees(mypose[2]) - 90) < 2.0:
            self.current_vlin = 0.0
            self.current_vrot = -0.1

        if abs(mypose[0] - 2.0) < 0.2 and \
        abs(mypose[1] - 3.0) < 0.2 and \
        abs(math.degrees(mypose[2]) - 0) < 2.0:
            self.current_vlin = 0.1
            self.current_vrot = 0.0

        if abs(mypose[0] - 8.0) < 0.2 and \
        abs(mypose[1] - 3.0) < 0.2 and \
        abs(math.degrees(mypose[2]) - 0) < 2.0:
            self.current_vlin = 0.0
            self.current_vrot = -0.1

        if abs(mypose[0] - 8.0) < 0.2 and \
        abs(mypose[1] - 3.0) < 0.2 and \
        abs(math.degrees(mypose[2]) - (-90)) < 2.0:
            self.current_vlin = 0.1
            self.current_vrot = 0.0

        if abs(mypose[0] - 8.0) < 0.2 and \
        abs(mypose[1] - (-3.0)) < 0.2 and \
        abs(math.degrees(mypose[2]) - (-90)) < 2.0:
            self.current_vlin = 0.0
            self.current_vrot = -0.1

        if abs(mypose[0] - 8.0) < 0.2 and \
        abs(mypose[1] - (-3.0)) < 0.2 and \
        abs(math.degrees(mypose[2]) - (-180)) < 2.0:
            self.current_vlin = 0.1
            self.current_vrot = 0.0


        """
        # LINEAR CONTROL STRATEGY
        # Compute delta_x and delta_y (distance to target position)
        delta_x = self.GOAL_POSITION[0] - mypose[0]
        delta_y = self.GOAL_POSITION[1] - mypose[1]

        # Compute angle and position to move
        E_th = math.atan(delta_y/delta_x)
        delta_th = E_th - mypose[2]

        if abs(delta_th) > 0.1:
            if delta_th > 0:
                self.current_vrot = 0.1
            else:
                self.current_vrot = -0.1
            self.current_vlin = 0.0
        else:
            self.current_vrot = 0.0
            self.current_vlin = 0.1
        """

        return (self.current_vlin, self.current_vrot)
