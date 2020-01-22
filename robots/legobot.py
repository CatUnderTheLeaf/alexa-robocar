#!/usr/bin/env python3
import sys
import _thread
import time
import numpy as np
from scripts.pose import Pose
from ev3dev2.sensor.lego import InfraredSensor
from ev3dev2.motor import OUTPUT_B, OUTPUT_C, OUTPUT_D, MoveDifferential, SpeedRPS, LargeMotor, MediumMotor
from ev3dev2.wheel import EV3Tire
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
from math import ceil, exp, sin, cos, tan, pi
from scripts.helpers import Struct

class LegoBot(MoveDifferential):
    
    def __init__(self, left_motor_port, right_motor_port, rot_motor_port,
            wheel_class, wheel_distance_mm,
            desc=None, motor_class=LargeMotor):
        MoveDifferential.__init__(self, left_motor_port, right_motor_port, wheel_class, wheel_distance_mm)
        """ 
        LegoBot Class inherits all usefull stuff for differential drive
        and adds sound, LEDs, IRSensor which is rotated by Medium Motor
         """
        self.leds = Leds()
        self.sound = Sound()
        self.leds.set_color("LEFT", "BLACK")
        self.leds.set_color("RIGHT", "BLACK")

        # Startup sequence
        self.sound.play_song((('C4', 'e'), ('D4', 'e'), ('E5', 'q')))
        self.leds.set_color("LEFT", "GREEN")
        self.leds.set_color("RIGHT", "GREEN")

        # create IR sensors
        self.ir_sensor = InfraredSensor()
        self.sensor_rotation_point = Pose( 0.05, 0.0, np.radians(0))
        self.sensor_rotation_radius = 0.04
        self.sensor_thread_run = False
        self.sensor_thread_id = None
        # temporary storage for ir readings and poses until half rotation is fully made
        self.ir_sensors = None   
                           
        # initialize motion
        self.ang_velocity = (0.0,0.0)
        self.rot_motor = MediumMotor(rot_motor_port)
        self.rotate_thread_run = False
        self.rotate_thread_id = None
        self.rotation_degrees = 180
       
        # information about robot for controller or supervisor
        self.info = Struct()
        self.info.wheels = Struct()       
        self.info.wheels.radius = self.wheel.radius_mm /1000
        self.info.wheels.base_length = wheel_distance_mm /1000         
        self.info.wheels.max_velocity = 2*pi*170/60  #  170 RPM
        self.info.wheels.min_velocity = 2*pi*30/60  #  30 RPM
        
        self.info.pose = None
         
        self.info.ir_sensors = Struct()
        self.info.ir_sensors.poses = None
        self.info.ir_sensors.readings = None
        self.info.ir_sensors.rmax = 0.7
        self.info.ir_sensors.rmin = 0.04

        # starting odometry thread
        self.odometry_start(0,0,0)
        # start measuring distance with IR Sensor in another thread while rotating
        self.sensor_update_start(self.rot_motor)
        # start rotating of medium motor
        self.rotate_and_update_sensors_start()
        



    def turn_off(self):        
        # stop odometry thread
        self.odometry_stop()
        # stop updating sensors
        self.rotate_and_update_sensors_stop()        
        # return robots head to start position 
        self.rot_motor.on_to_position(100, 0, True, True) 
        self.sensor_update_stop()
        # Shutdown sequence
        self.sound.play_song((('E5', 'e'), ('C4', 'e')))
        self.leds.set_color("LEFT", "BLACK")
        self.leds.set_color("RIGHT", "BLACK")


    def get_pose(self):
        """Get the pose of the object in world coordinates"""
        return Pose(self.x_pos_mm/1000, self.y_pos_mm/1000, self.theta)

    
    def move(self,dt):
        
        (vl, vr) = self.get_wheel_speeds()
        
        # actual robot move
        self.on_for_seconds(SpeedRPS(vl/2/pi), SpeedRPS(vr/2/pi), dt, False, False)
        
    def get_info(self):
        # getting updated info for supervisor
        self.info.pose = self.get_pose()
        return self.info
    
    def set_inputs(self,inputs):
        """ Setting new values of (vl, vr) sent by supervisor and controller """
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

    def sensor_update_start(self, motor, sleep_time=0.005):  # 5ms
        """
        A thread is started that will run until the user calls sensor_update_stop()
        which will set sensor_thread_run to False
        """
        self.ir_sensors = {}
        
        def _sensor_monitor():
            

            while self.sensor_thread_run:

                angle = -np.radians(motor.degrees) # convert from degrees to radians
                
                sensor_x = round(self.sensor_rotation_radius*cos(angle) + self.sensor_rotation_point.x, 3)
                sensor_y = round(self.sensor_rotation_radius*sin(angle) + self.sensor_rotation_point.y, 3)
                
                # adding to temp dict sensor readings and poses
                self.ir_sensors.update({(sensor_x, sensor_y, angle):round(self.ir_sensor.proximity*0.007, 3)})
                
                time.sleep(0.005)

            self.sensor_thread_id = None

        self.sensor_thread_run = True
        self.sensor_thread_id = _thread.start_new_thread(_sensor_monitor, ())

    def sensor_update_stop(self):
        """
        Signal the sensor update thread to exit and wait for it to exit
        """

        if self.sensor_thread_id:
            self.sensor_thread_run = False

            while self.sensor_thread_id:
                pass
        
            
    def rotate_and_update_sensors_start(self):
        
        self.info.ir_sensors.readings = []
        self.info.ir_sensors.poses = []
        
        def _rotate_monitor():            
             
            while self.rotate_thread_run:
                # writing ir sensor reading and poses from temp dict
                self.info.ir_sensors.readings = [*self.ir_sensors.values()]
                self.info.ir_sensors.poses = [*self.ir_sensors]
                # cleaning up temp dict
                self.ir_sensors = {}
                # rotate rotation motor with sensor
                self.rot_motor.on_for_degrees(50, self.rotation_degrees, True, True)
                time.sleep(0.005)
                # change orientation of rotation
                self.rotation_degrees = -self.rotation_degrees                
                
            self.rotate_thread_id = None

        self.rot_motor.position = 0
        # rotate head to the left at start
        self.rot_motor.on_for_degrees(100, -90, True, True)

        self.rotate_thread_run = True
        self.rotate_thread_id = _thread.start_new_thread(_rotate_monitor, ())


    def rotate_and_update_sensors_stop(self):
        """
        Signal the sensor update thread to exit and wait for it to exit
        """

        if self.rotate_thread_id:
            self.rotate_thread_run = False
            
            while self.rotate_thread_id:
                pass