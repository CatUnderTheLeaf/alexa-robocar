from scripts.controller import Controller
import math
import sys

class GoToAngle(Controller):
    """Go-to-angle steers the robot to a predefined angle in the world."""
    def __init__(self, params):
        """Initialize internal variables"""
        Controller.__init__(self,params)

    def set_parameters(self, params):
        """Set PID values
        
        The params structure is expected to have in the `gains` field three
        parameters for the PID gains.
        
        :param params.gains.kp: Proportional gain
        :type params.gains.kp: float
        :param params.gains.ki: Integral gain
        :type params.gains.ki: float
        :param params.gains.kd: Differential gain
        :type params.gains.kd: float
        """
        self.kp = params.pgain
               
    def execute(self, state, dt):
        """Calculate errors and steer the robot"""
     
        # The goal:
        theta_g = state.goal*math.pi/180
        
        # The robot:
        
        x_r, y_r, theta = state.pose
        # print("current Robot pose: ({}, {}, {})".format(x_r, y_r, theta), file=sys.stderr)
        
        # 1. Calculate simple proportional error
        error = (theta_g - theta + math.pi)%(2*math.pi) - math.pi
        # print("current err: ({})".format(error), file=sys.stderr)
        
        # 2. Calculate desired omega
        w_ = self.kp*error
        
        # 3. The linear velocity is given to us:
        v_ = state.velocity
        # print("current v, w: ({}, {})".format(v_, w_), file=sys.stderr)
        

        return [v_, w_]
