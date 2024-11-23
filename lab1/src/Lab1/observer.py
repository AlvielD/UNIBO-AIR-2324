import math

class Observer():
    def __init__(self):
        # Init parameters
        self.WHEEL_RADIUS = 0.0
        self.WHEEL_AXIS   = 0.0

        # Previous wheel encoders
        self.wl_0 = 0.0
        self.wr_0 = 0.0

        # Init pose coordinates (Gazebo sim)
        self.Ex = 3.0
        self.Ey = 0.0
        self.Eth = 0.0


    def init_pose (self, robot_pars):
        """
        Initialize the observer, using the given robot's parameters
        Return True if the inizialization was successful
        """
        self.WHEEL_RADIUS = robot_pars['wheel_radius']
        self.WHEEL_AXIS   = robot_pars['wheel_axis']
        return True


    def update_pose (self, wl, wr):
        """
        Incremental position estimation: update the current pose (x, y, th) of the robot
        taking into account the newly read positions (rotations) of the left and right wheels
        Returns the new robot's pose, as a triple (x, y, th)
        """
        # Compute encoders increment
        delt_wl = wl - self.wl_0
        delt_wr = wr - self.wr_0

        # Convert to wheel distance
        dl = delt_wl * self.WHEEL_RADIUS
        dr = delt_wr * self.WHEEL_RADIUS

        # Compute euclidean distance and angle
        d = (dr + dl)/2
        delta = (dr - dl)/self.WHEEL_AXIS

        # Compute distance on each axis
        dx = d*math.cos(delta/2)
        dy = d*math.sin(delta/2)

        # Update parameters
        self.Ex = self.Ex + dx*math.cos(self.Eth) - dy*math.sin(self.Eth)
        self.Ey = self.Ey + dx*math.sin(self.Eth) - dy*math.cos(self.Eth)
        self.Eth = self.Eth + delta

        # Update previous wheel encoders to current wheel encoders
        self.wl_0 = wl
        self.wr_0 = wr

        return (self.Ex, self.Ey, self.Eth)
