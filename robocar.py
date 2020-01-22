#!/usr/bin/env python3

import os
import sys
import time
import logging

from ev3dev2.sound import Sound
from ev3dev2.led import Leds
from robots.legobot import LegoBot
from supervisors.supervisor import LegoBotSupervisor
from ev3dev2.motor import OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor
from ev3dev2.wheel import EV3Tire

from agt import AlexaGadget

# set logger to display on both EV3 Brick and console
logging.basicConfig(level=logging.INFO, stream=sys.stdout, format='%(message)s')
logging.getLogger().addHandler(logging.StreamHandler(sys.stderr))
logger = logging.getLogger(__name__)


class MindstormsGadget(AlexaGadget):
    """
    An Mindstorms gadget that will react to the Alexa wake word.
    """

    def __init__(self):
        """
        Performs Alexa Gadget initialization routines and ev3dev resource allocation.
        """
        super().__init__()

        STUD_MM = 8
        DIST_BTW_WHEELS = 15 * STUD_MM

        self.bot = LegoBot(OUTPUT_B, OUTPUT_C, OUTPUT_D, EV3Tire, DIST_BTW_WHEELS)

    # def on_connected(self, device_addr):
    #     """
    #     Gadget connected to the paired Echo device.
    #     :param device_addr: the address of the device we connected to
    #     """
    #     self.leds.set_color("LEFT", "GREEN")
    #     self.leds.set_color("RIGHT", "GREEN")
    #     logger.info("{} connected to Echo device".format(self.friendly_name))

    # def on_disconnected(self, device_addr):
    #     """
    #     Gadget disconnected from the paired Echo device.
    #     :param device_addr: the address of the device we disconnected from
    #     """
    #     self.leds.set_color("LEFT", "BLACK")
    #     self.leds.set_color("RIGHT", "BLACK")
    #     logger.info("{} disconnected from Echo device".format(self.friendly_name))

    # def on_alexa_gadget_statelistener_stateupdate(self, directive):
    #     """
    #     Listens for the wakeword state change and react by turning on the LED.
    #     :param directive: contains a payload with the updated state information from Alexa
    #     """
    #     color_list = ['BLACK', 'AMBER', 'YELLOW', 'GREEN']
    #     for state in directive.payload.states:
    #         if state.name == 'wakeword':

    #             if state.value == 'active':
    #                 print("Wake word active", file=sys.stderr)
    #                 self.sound.play_song((('A3', 'e'), ('C5', 'e')))
    #                 for i in range(0, 4, 1):
    #                     self.leds.set_color("LEFT", color_list[i], (i * 0.25))
    #                     self.leds.set_color("RIGHT", color_list[i], (i * 0.25))
    #                     time.sleep(0.25)
    #                 self.bot.move(0.25)
    #             elif state.value == 'cleared':
    #                 print("Wake word cleared", file=sys.stderr)
    #                 self.sound.play_song((('C5', 'e'), ('A3', 'e')))
    #                 for i in range(3, -1, -1):
    #                     self.leds.set_color("LEFT", color_list[i], (i * 0.25))
    #                     self.leds.set_color("RIGHT", color_list[i], (i * 0.25))
    #                     time.sleep(0.25)


if __name__ == '__main__':

    gadget = MindstormsGadget()
    supervisor = LegoBotSupervisor(gadget.bot.get_pose(), gadget.bot.get_info())

    # Set LCD font and turn off blinking LEDs
    os.system('setfont Lat7-Terminus12x6')   

    # Gadget main entry point
    # gadget.main()
    
    tc = 1        
    for step in range(200):#200 
        #1. move robot
        gadget.bot.move(tc)        
        #2. supervisor calculates new velocities and we apply it to robot
        new_inputs = supervisor.execute(gadget.bot.get_info(), tc)
        gadget.bot.set_inputs(new_inputs)
    
    # Shutdown sequence
    gadget.bot.turn_off()
    
