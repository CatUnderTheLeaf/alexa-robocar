from scripts.supervisor import Supervisor
from scripts.helpers import Struct
from scripts.pose import Pose
from math import pi, sin, cos, log1p, copysign, sqrt
import numpy
import sys

class LegoBotSupervisor(Supervisor):
    """
       The LegoBotSupervisor object implements the system functions
       necessary to operate a LegoBot, namely the uni2diff unicycle to differential
       motion model conversion, the Jacobian problem, and any other computationally complex interface.
    """
       
    def __init__(self, robot_pose, robot_info):
        """Initialize internal variables"""
        Supervisor.__init__(self, robot_pose, robot_info)

        # Create & set the controller
        # self.current = self.create_controller('GoToAngle', self.parameters)
        # self.gtg = self.create_controller('GoToGoal', self.parameters)
        self.ao = self.create_controller('AvoidObstacles', self.parameters)

        # Set the controller
        self.current = self.ao

                    
    def init_default_parameters(self):
        """Sets the default PID parameters, goal, and velocity"""
        
        p = Struct()
        # for Avoid Obstacle testing no need for goal
        # p.goal = Struct()
        # p.goal.x = 0.5 #m
        # p.goal.y = 0.5 #m
        p.velocity = 0.2 #m/sec
        p.gains = Struct()
        p.gains.kp = 4.0
        p.gains.ki = 0.2
        p.gains.kd = 0.0
        
        self.parameters = p
        
    def set_parameters(self,params):
        """Set parameters for itself and the controllers"""
        # self.parameters.goal = params.goal
        self.parameters.velocity = params.velocity
        # self.parameters.pgain = params.pgain # for goToAngle controller
        self.current.set_parameters(self.parameters)
        
                                  
    def uni2diff(self,uni):
        """Convert from unicycle model to differential model"""
        (v,w) = uni
        
        R = self.robot.wheels.radius 
        L = self.robot.wheels.base_length

        vl = (2*v-w*L)/(2*R)
        vr = (2*v+w*L)/(2*R)
        
        return (vl,vr)
            
    def estimate_pose(self):
        """Update self.pose_est using odometry"""
        
        x_r, y_r, theta = self.robot.pose
        
        return self.robot.pose

    def execute(self, robot_info, dt):
        """Inherit default supervisor procedures and return unicycle model output (x, y, theta)"""
        # for gotogoal
        # if not self.at_goal():
        #     output = Supervisor.execute(self, robot_info, dt)
        #     return self.ensure_w(self.uni2diff(output))
        # else:
        #     return 0,0

        # testing avoidObstacles
        output = Supervisor.execute(self, robot_info, dt)
        return self.ensure_w(self.uni2diff(output))


    def process_state_info(self, state):
        """Update state parameters for the controllers and self"""

        Supervisor.process_state_info(self,state)
        self.parameters.robot = self.robot
        self.parameters.pose = self.pose_est
                                       

    def get_controller_state(self):
        return self.parameters
    
    def at_goal(self):
        distance_to_goal = sqrt( \
                                (self.pose_est.x - self.parameters.goal.x)**2
                              + (self.pose_est.y - self.parameters.goal.y)**2)
        print("dist to goal: ({})".format(distance_to_goal), file=sys.stderr)
        
        return distance_to_goal < 0.02

    def ensure_w(self,v_lr):        
        """ 
            The robot’s motors have a maximum angular velocity, and the motors stall at low speeds. 
            Suppose that we pick a linear velocity v that requires the motors to spin at 90% power. 
            Then, we want to change ω from 0 to some value that requires 20% more power from the right motor, 
            and 20% less power from the left motor. This is not an issue for the left motor, 
            but the right motor cannot turn at a capacity greater than 100%. 
            The results is that the robot cannot turn with the ω specified by our controller.

            Since PID controllers focus more on steering than on controlling the linear velocity, 
            we want to prioritize ω over v in situations, where we cannot satisfy ω with the motors. 
            In fact, we will simply reduce v until we have sufficient headroom to achieve ω with the robot. 
            The function is designed to ensure that ω is achieved even if the original combination of v and ω exceeds the maximum vl and vr.
        """     
        # This code is taken directly from Sim.I.Am project
        v_max = self.robot.wheels.max_velocity
        v_min = self.robot.wheels.min_velocity
       
        R = self.robot.wheels.radius 
        L = self.robot.wheels.base_length 
        
        def diff2uni(vl,vr):
            return (vl+vr) * R/2, (vr-vl) * R/L
        
        v, w = diff2uni(*v_lr)
        
        if v == 0:
            
            # Robot is stationary, so we can either not rotate, or
            # rotate with some minimum/maximum angular velocity

            w_min = R/L*(2*v_min)
            w_max = R/L*(2*v_max)
            
            if abs(w) > w_min:
                w = copysign(max(min(abs(w), w_max), w_min), w)
            else:
                w = 0
            
            return self.uni2diff((0,w))
            
        else:
            # 1. Limit v,w to be possible in the range [vel_min, vel_max]
            # (avoid stalling or exceeding motor limits)
            v_lim = max(min(abs(v), (R/2)*(2*v_max)), (R/2)*(2*v_min))
            w_lim = max(min(abs(w), (R/L)*(v_max - v_min)), 0)
            
            # 2. Compute the desired curvature of the robot's motion
            
            vl,vr = self.uni2diff((v_lim, w_lim))
            
            # 3. Find the max and min vel_r/vel_l
            v_lr_max = max(vl, vr)
            v_lr_min = min(vl, vr)
            
            # 4. Shift vr and vl if they exceed max/min vel
            if (v_lr_max > v_max):
                vr -= v_lr_max - v_max
                vl -= v_lr_max - v_max
            elif (v_lr_min < v_min):
                vr += v_min - v_lr_min
                vl += v_min - v_lr_min
            
            # 5. Fix signs (Always either both positive or negative)
            v_shift, w_shift = diff2uni(vl,vr)
            
            v = copysign(v_shift,v)
            w = copysign(w_shift,w)
            
            return self.uni2diff((v,w))