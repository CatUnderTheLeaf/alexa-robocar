from controllers.pid_controller import PIDController
import math
import numpy
from scripts.pose import Pose
import sys

class AvoidObstacles(PIDController):
    """Avoid obstacles is a controller that checks the sensors
       for any readings, constructs 'obstacle' vectors and directs the robot
       in the direction of their weighted sum."""
    def __init__(self, params):
        '''Initialize internal variables'''
        PIDController.__init__(self,params)

    def set_parameters(self, params):
        """
        Set parameters.
        """
        PIDController.set_parameters(self,params)

    def get_heading(self, state):
        """Get the direction away from the obstacles as a vector."""
        # print("current Robot IR readings: ({})".format(state.robot.ir_sensors.readings), file=sys.stderr)
        # for pose in state.robot.ir_sensors.poses:
        #     print("IR poses: ({})".format(pose), file=sys.stderr)
        # Calculate heading:
        ws = sum(state.robot.ir_sensors.readings)
        self.weights = [w / ws for w in state.robot.ir_sensors.readings]
        x, y = 0, 0
        for d,p,w in zip(state.robot.ir_sensors.readings, state.robot.ir_sensors.poses, self.weights):
            pose = Pose(d) >> Pose(p)
            x += pose.x*w
            y += pose.y*w                
        # print("current Robot heading: ({}, {})".format(x, y), file=sys.stderr)
        
        return numpy.array([x, y, 1])
    
    def execute(self, state, dt):
        
        v, w = PIDController.execute(self, state, dt)
        # TODO check this regulation of velocity
        # dmin = min(state.sensor_distances)
        # v *= ((dmin - 0.04)/0.26)**2
        
        return v, w    