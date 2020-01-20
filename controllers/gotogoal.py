from controllers.pid_controller import PIDController
import math
import numpy
import sys

class GoToGoal(PIDController):
    """Go-to-goal steers the robot to a predefined position in the world."""
    def __init__(self, params):
        """Initialize internal variables"""
        PIDController.__init__(self,params)

    # Let's overwrite this way:
    def get_heading_angle(self, state):
        """Get the direction from the robot to the goal as a vector."""
        
        # The goal:
        x_g, y_g = state.goal.x, state.goal.y
        
        # The robot:
        x_r, y_r, theta = state.pose

        # Where is the goal in the robot's frame of reference?
        return (math.atan2(y_g - y_r, x_g - x_r) - theta + math.pi)%(2*math.pi) - math.pi

    def get_heading(self,state):
        """ 
        Get the direction in which the controller wants to move the robot
        as a vector in the robot's frame of reference.
         """
        goal_angle = self.get_heading_angle(state)
        return numpy.array([math.cos(goal_angle),math.sin(goal_angle),1])
    
    def execute(self, state, dt):
        
        v, w = PIDController.execute(self, state, dt)
# #TODO check this
# velocity should depend on distance to the goal        
        # The goal:
#         x_g, y_g = state.goal.x, state.goal.y
        
#         # The robot:
#         x_r, y_r, theta = state.pose
#         print("current Robot pose: ({}, {}, {})".format(x_r, y_r, theta), file=sys.stderr)
        
#         # distance between goal and robot in x - direction
#         u_x = x_g - x_r

#         # distance between goal and robot in y - direction
#         u_y = y_g - y_r

#         # distance  between robot and goal
#         # dist = numpy.linalg.norm(numpy.array([u_x,u_y]))
#         # v = abs(w) + dist
#         v = state.velocity
        # 
        print("current v, w: ({}, {})".format(v, w), file=sys.stderr)
        
        return v, w