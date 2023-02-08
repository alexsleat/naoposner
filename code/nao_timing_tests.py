#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  9 08:16:07 2022

@author: tims
"""

#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Script to test all the functionalities of the CommandExecuterModule
"""

# For stimuli
import os
import numpy as np
import time
import cv2

# For threading
import logging
import threading
import time

# ROS
import rospy
from std_msgs.msg import String

#
from optparse import OptionParser
import configparser

# For testing generate random numbers
from random import randrange

# For Operation of NAO
import sys
import time
import pinger

# Requires certain setup (See GitSVN)
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

import itertools, random

# To logg correct timestamps
from datetime import datetime

PSYCHOPY_FLAG = True
DEBUG_FLAG = True
NAO_FLAG = True

if PSYCHOPY_FLAG:
    pass
else:
    # Read dummy images for stimuli
    blank = 255 * np.ones((512,512,3), np.uint8)
    #img0 = np.zeros((400, 400, 3), dtype = "uint8")

    path0 = os.getcwd() + "/letter-t-512.jpg"
    letter_t = cv2.imread(path0, 0)

    path1 = os.getcwd() + "/letter-v-512.jpg"
    letter_v = cv2.imread(path1, 0)

## Read the configurables from the config file
config = configparser.ConfigParser()
config.read('config.ini')

# If working with the real robot, we need to adjust this
NAO_IP = str(config['NAO']['Ip'])
NAO_PORT = int(config['NAO']['Port'])

print(NAO_IP, type(NAO_IP))
print(NAO_PORT, type(NAO_PORT))

# Gloabl Variables to store the modules
CommandExecuter = None
memory = None

# Scale how much time should pass between each stimuli/rotation cycle
# Offset + Wait is duration between stimuli
offset = 0.1
wait = 0.2

# That is the duration of how long it takes to rotate NAOS head
duration = 0.7
# Delay determines how much ms are between head movement and stimuli presentation
delay = 0.3

# Coordinates [In Torso Frame] for the left-, right screen and the rest position
# Finetune with the real setup using choreograph and then adjust here
left_screen = {"x":1, "y":1, "z":0}
right_screen = {"x":1, "y":-1, "z":0}
rest_position = {"x":1, "y":0, "z":0.1}

# Controlls for how long the system waits until the image is closed
# Finetune with practical setup, default 1 ms
time_stimulus_is_visible = 1

class CommandExecuterModule(ALModule):
    """Counts seen Faces"""

    def __init__(self,name):

        # Change for head turning speed:
        self.maxSpeed = 0.3

        self.x = 1.0
        self.y = 0.0
        self.z = 0.1

        self.old_x = 0.0
        self.old_y = 0.0
        self.old_z = 0.0

        self.alive = True

        ALModule.__init__(self,name)
        # No need for IP Adress bc PyBroker connected to Naoqi PyBroker

        # Create Proxy ALtts for letting NAO speak
        self.tts = ALProxy("ALTextToSpeech") # No IP Adress or ports needed
        self.posture = ALProxy("ALRobotPosture")
        # The tracker module is for coordinate related movements
        self.tracker = ALProxy( "ALTracker" )
        # Enbales to run pre-installed behaviours
        self.bm = ALProxy("ALBehaviorManager")
        # Enables Playback of soundfiles
        self.framemanager = ALProxy("ALFrameManager")
        self.player = ALProxy('ALAudioPlayer')
        self.playerStop = ALProxy('ALAudioPlayer', True) #Create another proxy as wait is blocking if audioout is remote

        self.led_controller = ALProxy("ALLeds")
        self.flash_duration = 2 # Adjustable
        self.delay_between_trials = 1 # Setbased on Lucas preferences

        global memory
        memory = ALProxy("ALMemory")

        # Tell the robot to "Crouch", stops over heating. Can be changed to "StandInit" if required for experiments
        #self.posture.goToPosture("Crouch", 1.0)
        time.sleep(3)

        # ROS Pubs n subs:
        self.pub_stimulus = rospy.Publisher('stimulus', String, queue_size=0)
        self.pub_logger = rospy.Publisher('logger', String, queue_size=0)

        self.sub_keypress = rospy.Subscriber('keypress', String, self.keypressCb)
        rospy.init_node('nao_cueing', anonymous=True)

    def keypressCb(self, data):
        print(data.data)
        # @TODO add logging here for key presss

    def updateCoordinates(self,x,y,z):
        """Function that simply updates Parameters"""
        self.x = x
        self.y = y
        self.z = z

    def onCallLook(self):
        """Lets NAO look to a certain Position"""

        # TODO: Adept Parameters to be variable
        # Write a function, which sets the Parameters for those kind of movements
        # self.updateCoordinates(1.0, -2.5, 0.8)

        while(self.alive):

            if self.y != self.old_y:

                self.old_x = self.x
                self.old_y = self.y
                self.old_z = self.z

                print("Head turning: {},{},{}".format(str(self.x), str(self.y), str(self.z)))

                self.useWholeBody = False
                self.frame = 0 #0 - TORSO, 1 - World, 2- Robot
                now = datetime.now()
                head_info1 = now.strftime("%d.%m.%y-%Hh%Mm%Ss%fns")
                # head_logger = "Head turned at: {}".format(str(head_info))
                # self.pub_logger.publish(head_logger)

                self.tracker.lookAt([self.x, self.y, self.z], self.frame, self.maxSpeed, self.useWholeBody)
                now = datetime.now()
                head_info2 = now.strftime("%d.%m.%y-%Hh%Mm%Ss%fns")
                head_logger =  "HeadTurn,{},{}".format(str(head_info1), str(head_info2))
                self.pub_logger.publish(head_logger)

    def flash_eyes(self, color=None):

        sGroup = "FaceLeds"

        if color == "blue":
            duration = self.delay_between_trials
        else:
            duration = self.flash_duration
        p = color

        self.ids = []
        self.leds = ALProxy("ALLeds")
        sGroup = "FaceLeds"

        #id = self.leds.post.fadeRGB(sGroup, float(p[0])/255, float(p[1])/255, float(p[2])/255, duration)
        id = self.leds.post.fadeRGB(sGroup, p, duration)
        self.ids.append(id)
        self.leds.wait(id, 0)
        self.ids.remove(id)

    def set_eyes(self, state=True):

        sGroup = "FaceLeds"
        if state:
            self.led_controller.on(sGroup)
        else:
            self.led_controller.off(sGroup)


# Main Function for NaoPosner Experiment
class NaoPosnerExperiment():

    def __init__(self, naoip, naoport):

        # Name must match variable name
        self.myBroker = ALBroker("myBroker",
        "0.0.0.0",
        0,
        naoip,
        naoport)

        self.CommandExecuter = CommandExecuterModule("CommandExecuter")

        self.block_ready_color = "green"
        self.trial_ready_color = "blue"
        self.trial_number = 1
        self.block_type = "N/A"

    def perform_test(self, num_of_runs, time_between=1):

        print("Eyes off")
        self.CommandExecuter.set_eyes(False)

        pinger.ping(NAO_IP)

        for i in range(num_of_runs):

            print(i, "Eyes on", str(pinger.ping(NAO_IP)))
            self.CommandExecuter.set_eyes(True)
            self.CommandExecuter.pub_stimulus.publish("t,t")
            time.sleep(time_between)

            print(i, "Eyes off", str(pinger.ping(NAO_IP)))
            self.CommandExecuter.pub_stimulus.publish(" , ")
            self.CommandExecuter.set_eyes(False)
            time.sleep(time_between)
            


    # Nao Rest
    ## Send to neutral pose
    def nao_rest(self):
            # self.CommandExecuter.tracker.lookAt([rest_position["x"], rest_position["y"], rest_position["z"]], 0, self.CommandExecuter.maxSpeed, False)
            self.CommandExecuter.updateCoordinates(rest_position["x"], rest_position["y"], rest_position["z"])
            self.CommandExecuter.pub_stimulus.publish(" , ")

    # Nao Move
    ## Send to the correct position, and send sequence to display to psychopy
    def nao_move(self, t):

        dir_to_look = (right_screen["x"], right_screen["y"], right_screen["z"])
        str_to_display = "N/A"

        # Check Left direction and congruency:
        if(t[0] == "L"):
            if(t[2] != "C"):
                dir_to_look = (left_screen["x"], left_screen["y"], left_screen["z"])
            # Check what value to display:
            if(t[1] == "T"):
                str_to_display = "t, "
            else:
                str_to_display = "v, "
        # Check Right direction and congruency:
        else:
            if(t[2] == "C"):
                dir_to_look = (left_screen["x"], left_screen["y"], left_screen["z"])
            # Check what value to display:
            if(t[1] == "T"):
                str_to_display = " ,t"
            else:
                str_to_display = " ,v"

        # update coordinates, and publish the stimuli
        self.CommandExecuter.updateCoordinates(dir_to_look[0], dir_to_look[1], dir_to_look[2] )
        # self.CommandExecuter.tracker.lookAt([dir_to_look[0], dir_to_look[1], dir_to_look[2]], 0, self.CommandExecuter.maxSpeed, False)

        self.CommandExecuter.pub_stimulus.publish(str_to_display)


if __name__ == '__main__':
    #main()
    e = NaoPosnerExperiment(NAO_IP, NAO_PORT)
    
    e.perform_test(50, 0.5)
    e.nao_rest()
