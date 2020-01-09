#
# (c) PySimiam Team 2014
#
# Contact person: John Witten <jon.witten@gmail.com>
#
# This class was implemented as a weekly programming excercise
# of the 'Control of Mobile Robots' course by Magnus Egerstedt.
#
from scripts.supervisor import Supervisor
from scripts.helpers import Struct
from scripts.pose import Pose
from math import pi, sin, cos, log1p, copysign, sqrt
import numpy
import sys

class LegoBotSupervisor(Supervisor):
    """The LegoBotSupervisor inherits from the superclass 'supervisor.Supervisor'
       to implement detailed calculations for any inheriting LegoBot supervisor.
       Students are intended to inherit from this class when making their own supervisors.
       

       Most importantly, the LegoBotSupervisor object implements the system functions
       necessary to operate a LegoBot, namely the uni2diff unicycle to differential
       motion model conversion, the Jacobian problem, and any other computationally complex interface.

       """

    # ir_coeff = numpy.array([ 8.56495710e-18,  -3.02930608e-14,
    #                       4.43025017e-11,  -3.49052288e-08,
    #                       1.61452174e-05,  -4.44025236e-03,
    #                       6.74137385e-1])
       
    def __init__(self, robot_pose, robot_info):
        """Initialize internal variables"""
        Supervisor.__init__(self, robot_pose, robot_info)

        # Create & set the controller
        # self.current = self.create_controller('GoToAngle', self.parameters)
        self.gtg = self.create_controller('GoToGoal', self.parameters)

        # Set the controller
        self.current = self.gtg

                    
    def init_default_parameters(self):
        """Sets the default PID parameters, goal, and velocity"""
        
        p = Struct()
        p.goal = Struct()
        p.goal.x = 0.5 #m
        p.goal.y = 0.5 #m
        p.velocity = 0.2 #m/sec
        p.gains = Struct()
        p.gains.kp = 4.0
        p.gains.ki = 0.2
        p.gains.kd = 0.0
        
        self.parameters = p
        
    def set_parameters(self,params):
        """Set parameters for itself and the controllers"""
        self.parameters.goal = params.goal
        self.parameters.velocity = params.velocity
        self.parameters.pgain = params.pgain
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
         
    
    # def get_ir_distances(self):
    #     """Converts the IR distance readings into a distance in meters"""
        
    #     #Insert Week 2 Assignment Code Here
    #     ir_coeff = numpy.array([1.16931064e+07, -1.49425626e+07,
    #                             7.96904053e+06, -2.28884314e+06,
    #                             3.80068213e+05, -3.64435691e+04,
    #                             1.89558821e+03])
    #     # ir_distances = [0]*len(self.robot.ir_sensors.readings) #populate this list
    #     ir_distances = numpy.polyval(ir_coeff, self.robot.ir_sensors.readings)
    #     #End Assignment week2

    #     return ir_distances

    def execute(self, robot_info, dt):
        """Inherit default supervisor procedures and return unicycle model output (x, y, theta)"""
        
        if not self.at_goal():
            output = Supervisor.execute(self, robot_info, dt)
            return self.ensure_w(self.uni2diff(output))
        else:
            return 0,0


    def process_state_info(self, state):
        """Update state parameters for the controllers and self"""

        Supervisor.process_state_info(self,state)
        
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
      
        # This code is taken directly from Sim.I.Am week 4
        # I'm sure one can do better. 

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