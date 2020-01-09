#!/usr/bin/env python3
# (c) PySimiam Team 2014
#
# Contact person: Tim Fuchs <typograph@elec.ru>
#
# This class was implemented as a weekly programming excercise
# of the 'Control of Mobile Robots' course by Magnus Egerstedt.
#
import sys
import _thread
import numpy as np
from scripts.pose import Pose
# from sensor import ProximitySensor
from ev3dev2.sensor.lego import InfraredSensor
from ev3dev2.motor import OUTPUT_B, OUTPUT_C, OUTPUT_D, MoveDifferential, SpeedRPM, SpeedPercent, SpeedNativeUnits, SpeedRPS, SpeedDPS, SpeedDPM, LargeMotor, MediumMotor
from ev3dev2.wheel import EV3Tire
from math import ceil, exp, sin, cos, tan, pi
from scripts.helpers import Struct

# class LegoBot_IRSensor(ProximitySensor):
#     """Inherits from the proximity sensor class. Performs calculations specific to the khepera3 for its characterized proximity sensors"""
    
#     ir_coeff = np.array([  1.16931064e+07,  -1.49425626e+07, \
#                            7.96904053e+06,  -2.28884314e+06, \
#                            3.80068213e+05,  -3.64435691e+04, \
#                            1.89558821e+03])
    
#     def __init__(self,pose,robot):
#         # values copied from SimIAm    
#         ProximitySensor.__init__(self, pose, robot, (0.04, 0.3, np.radians(6)))

#     def distance_to_value(self,dst):
#         """Returns the distance calculation from the distance readings of the proximity sensors""" 
        
#         if dst < self.rmin :
#             return 917
#         elif dst > self.rmax:
#             return 133
#         else:
#             return np.polyval(self.ir_coeff,dst)

class LegoBot(MoveDifferential):
    
    def __init__(self, left_motor_port, right_motor_port, rot_motor_port,
            wheel_class, wheel_distance_mm,
            desc=None, motor_class=LargeMotor):
        MoveDifferential.__init__(self, left_motor_port, right_motor_port, wheel_class, wheel_distance_mm)
        
        # self.rot_motor = LargeMotor(rot_motor_port)
        self.rot_motor = MediumMotor(rot_motor_port)
        # create IR sensors
        self.ir_sensor = InfraredSensor()
        self.ir_sensors = []
        self.sensor_thread_run = False
        self.sensor_thread_id = None      
        ir_sensor_poses = [
                          Pose( 0.0, 0.0, np.radians(90)),
                          Pose( 0.0, 0.0, np.radians(45)),
                          Pose( 0.0, 0.0, np.radians(0)),
                          Pose( 0.0, 0.0, np.radians(-45)),
                          Pose( 0.0, 0.0, np.radians(-90))
                          ]                          
                           
        # for pose in ir_sensor_poses:
        #     self.ir_sensors.append(QuickBot_IRSensor(pose,self))
                                
        # initialize motion
        self.ang_velocity = (0.0,0.0)

        self.info = Struct()
        self.info.wheels = Struct()
        # # these were the original parameters

        self.info.wheels.radius = self.wheel.radius_mm /1000
        self.info.wheels.base_length = wheel_distance_mm /1000
         
        self.info.wheels.max_velocity = 2*pi*170/60  #self.max_speed # 170 RPM
        self.info.wheels.min_velocity = 2*pi*30/60  #  30 RPM

        self.info.pose = None

        # self.rot_motor.on_to_position(100, -90, True, True) #rotate head to the left
        self.rot_motor.on_for_degrees(100, -90)
        
        # self.info.ir_sensors = Struct()
        # self.info.ir_sensors.poses = ir_sensor_poses
        # self.info.ir_sensors.readings = None
        # self.info.ir_sensors.rmax = 0.3
        # self.info.ir_sensors.rmin = 0.04

    def get_pose(self):
        """Get the pose of the object in world coordinates"""
        return Pose(self.x_pos_mm/1000, self.y_pos_mm/1000, self.theta)

    
    def move(self,dt):
        
        (vl, vr) = self.get_wheel_speeds()
        #actual robot move
        self.on_for_seconds(SpeedRPS(vl/2/pi), SpeedRPS(vr/2/pi), dt, False, False)
        
    def get_info(self):
        # self.update_sensors()
        # self.info.ir_sensors.readings = [sensor.reading() for sensor in self.ir_sensors]
        self.info.pose = self.get_pose()
        return self.info
    
    def set_inputs(self,inputs):
        self.set_wheel_speeds(inputs)
        
    def get_wheel_speeds(self):
        return self.ang_velocity
    
    def set_wheel_speeds(self,*args):
        if len(args) == 2:
            (vl, vr) = args
        else:
            (vl, vr) = args[0]
            
        left_ms  = max(-self.info.wheels.max_velocity, min(self.info.wheels.max_velocity, vl))
        right_ms = max(-self.info.wheels.max_velocity, min(self.info.wheels.max_velocity, vr))

        self.ang_velocity = (left_ms, right_ms)

    # def get_external_sensors(self):
    #     return self.ir_sensors

    def sensor_update_start(self, motor, sensor, sleep_time=0.005):  # 5ms
        """
        A thread is started that will run until the user calls sensor_update_stop()
        which will set sensor_thread_run to False
        """

        def _sensor_monitor():
            

            while self.sensor_thread_run:

                #TODO
                self.ir_sensors.append((motor.position, sensor.proximity))

                # if sleep_time:
                #     time.sleep(sleep_time)

            self.sensor_thread_id = None

        self.sensor_thread_run = True
        self.sensor_thread_id = _thread.start_new_thread(_sensor_monitor, ())

    def sensor_update_stop(self):
        """
        Signal the sensor update thread to exit and wait for it to exit
        """

        print("position of sensor: {}".format(self.ir_sensors), file=sys.stderr)

        if self.sensor_thread_id:
            self.sensor_thread_run = False

            while self.sensor_thread_id:
                pass
        self.ir_sensors = []
            
    def rotate_and_update_sensors(self):
        
        self.sensor_update_start(self.rot_motor, self.ir_sensor)    
        # print("position of motor: {}".format(self.rot_motor.position), file=sys.stderr)
        # self.rot_motor.on_to_position(100, 90, True, True)
        self.rot_motor.on_for_degrees(100, 90, True, True)
        # while self.rot_motor.is_running:
        #     print("position of motor: {}".format(self.rot_motor.position), file=sys.stderr)
        #     print("sensor data: {}".format(self.ir_sensor.proximity), file=sys.stderr)
        # print("position of motor: {}".format(self.rot_motor.position), file=sys.stderr)
        # self.rot_motor.on_to_position(100, -90, True, True)
        self.rot_motor.on_for_degrees(100, -90, True, True)
        # while self.rot_motor.is_running:
        #     print("sensor data: {}".format(self.ir_sensor.proximity), file=sys.stderr)
        
        # print("position of motor: {}".format(self.rot_motor.position), file=sys.stderr)
        # self.rot_motor.on_to_position(100, 0)

    #     for sensor in self.ir_sensors:
    #         sensor.update_distance()
        self.sensor_update_stop()
    