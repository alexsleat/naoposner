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

# Requires certain setup (See GitSVN)
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

import itertools, random

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
offset = 0.8
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

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

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


        global memory
        memory = ALProxy("ALMemory")

        # Tell the robot to "Crouch", stops over heating. Can be changed to "StandInit" if required for experiments
        self.posture.goToPosture("Crouch", 1.0)

        # ROS Pubs n subs:
        self.pub_stimulus = rospy.Publisher('stimulus', String, queue_size=0)
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

        self.useWholeBody = False
        self.frame = 0 #0 - TORSO, 1 - World, 2- Robot

        self.tracker.lookAt([self.x, self.y, self.z], self.frame, self.maxSpeed, self.useWholeBody)

# def main():
#     """Main entry Point"""

#     # Name must match variable name
#     myBroker = ALBroker("myBroker",
#     "0.0.0.0",
#     0,
#     NAO_IP,
#     NAO_PORT)

#     # Name must match variable name
#     global CommandExecuter
#     CommandExecuter = CommandExecuterModule("CommandExecuter")

#     if PSYCHOPY_FLAG:
#         CommandExecuter.pub_stimulus.publish(" , ")
#     else:
#         cv2.imshow('Left Screen' , blank)
#         cv2.imshow('Right Screen', blank)
#         cv2.waitKey(1)

#     # Plan to insert a stop here until the images are in positions on the Screens
#     # Continue with user Input, signalling "Ready"
#     # raw_input("Press Enter to continue...") # Waits for Enter to be pressed but is not robust

#     # Perform a certain amount of trials
#     for i in range(10):

#         arg = randrange(8)

#         try:
#             # Define a thread parallel to the main one
#             # Thread runs the function that controlls NAOs head
#             t1 = threading.Timer(offset, CommandExecuter.onCallLook)
#         except KeyboardInterrupt:
#             print ("Interrupted by us ... Shutdown")

#         # At the beginning of the loop start the thread
#         t1.start()
#         # Wait a given time until updating the screens
#         # To get the behavior wanted: Spin Head and then Stimulus
#         time.sleep(offset + duration + delay)

#         # The seven possible positions
#         if arg == 0:
#             CommandExecuter.updateCoordinates(left_screen["x"], left_screen["y"], left_screen["z"])
#             if PSYCHOPY_FLAG:
#                 CommandExecuter.pub_stimulus.publish("t, ")
#             else:
#                 cv2.imshow('Left Screen' , letter_t)
#                 cv2.imshow('Right Screen', blank)
#         if arg == 1:
#             CommandExecuter.updateCoordinates(right_screen["x"], right_screen["y"], right_screen["z"])
#             if PSYCHOPY_FLAG:
#                 CommandExecuter.pub_stimulus.publish("t, ")
#             else:
#                 cv2.imshow('Left Screen', letter_t)
#                 cv2.imshow('Right Screen', blank)
#         if arg == 2:
#             CommandExecuter.updateCoordinates(left_screen["x"], left_screen["y"], left_screen["z"])
#             if PSYCHOPY_FLAG:
#                 CommandExecuter.pub_stimulus.publish(" ,t")
#             else:
#                 cv2.imshow('Left Screen', blank)
#                 cv2.imshow('Right Screen', letter_t)
#         if arg == 3:
#             CommandExecuter.updateCoordinates(right_screen["x"], right_screen["y"], right_screen["z"])
#             if PSYCHOPY_FLAG:
#                 CommandExecuter.pub_stimulus.publish(" ,t")
#             else:
#                 cv2.imshow('Left Screen' , blank)
#                 cv2.imshow('Right Screen', letter_t)
#         if arg == 4:
#             CommandExecuter.updateCoordinates(left_screen["x"], left_screen["y"], left_screen["z"])
#             if PSYCHOPY_FLAG:
#                 CommandExecuter.pub_stimulus.publish("v, ")
#             else:
#                 cv2.imshow('Left Screen' , letter_v)
#                 cv2.imshow('Right Screen', blank)
#         if arg == 5:
#             CommandExecuter.updateCoordinates(right_screen["x"], right_screen["y"], right_screen["z"])
#             if PSYCHOPY_FLAG:
#                 CommandExecuter.pub_stimulus.publish("v, ")
#             else:
#                 cv2.imshow('Left Screen', letter_v)
#                 cv2.imshow('Right Screen', blank)
#         if arg == 6:
#             CommandExecuter.updateCoordinates(left_screen["x"], left_screen["y"], left_screen["z"])
#             if PSYCHOPY_FLAG:
#                 CommandExecuter.pub_stimulus.publish(" ,v")
#             else:
#                 cv2.imshow('Left Screen', blank)
#                 cv2.imshow('Right Screen', letter_v)
#         if arg == 7:
#             CommandExecuter.updateCoordinates(right_screen["x"], right_screen["y"], right_screen["z"])
#             if PSYCHOPY_FLAG:
#                 CommandExecuter.pub_stimulus.publish(" ,v")
#             else:
#                 cv2.imshow('Left Screen' , blank)
#                 cv2.imshow('Right Screen', letter_v)

#         # cv2.waitKey(n) will display a frame for n ms, after which display will be automatically closed.
#         # This is also a variable that we have to tune for a practical trial
#         cv2.waitKey(time_stimulus_is_visible)

#         # After loop wait 500 ms (0.5s)before returning to rest position
#         # From what I got from the instructions, this block has to be in the loop above
#         # S.t. NAO Turns head -> Stimulus appears -> NAO returns to restposition
#         time.sleep(0.5)
#         CommandExecuter.tracker.lookAt([rest_position["x"], rest_position["y"], rest_position["z"]], 0, CommandExecuter.maxSpeed, False)
#         # print(arg)

#         # Clear the Screens to prepare for new trial
#         if PSYCHOPY_FLAG:
#             CommandExecuter.pub_stimulus.publish(" , ")
#         else:
#             cv2.imshow('Left Screen' , blank)
#             cv2.imshow('Right Screen', blank)
#             # cv2.waitKey(1)

#         # Wait 1s for the keypress to be processed
#         time.sleep(1.0)

#     # For dummy programm clear screens and end programm after one iteration
#     # For the real deal we grab the vriables for the next block/trial from psychopy here
#     if PSYCHOPY_FLAG:
#         CommandExecuter.pub_stimulus.publish("end,end")
#     else:
#         cv2.destroyAllWindows()
#     #myBroker.shutdown()
#     sys.exit(0)


# Main Function for Experiment
class Experiment():

    def __init__(self, naoip, naoport):

        # Name must match variable name
        self.myBroker = ALBroker("myBroker",
        "0.0.0.0",
        0,
        naoip,
        naoport)

        self.CommandExecuter = CommandExecuterModule("CommandExecuter")

    # Play block:
    ## Alternative method to Main:
    def play_block(self, warmup = True, num_blocks = 5):

        self.CommandExecuter.pub_stimulus.publish(" , ")
        time.sleep(1)

        # Generate & randomize the full block:
        list_block = self.generate_all_block(warmup, num_blocks)

        # Run through a full trial:
        counter = 0
        trial_num = 1
        for t in (list_block):

            # At the beginning of the loop start the thread
            try:
                # Define a thread parallel to the main one
                # Thread runs the function that controlls NAOs head
                t1 = threading.Timer(offset, self.CommandExecuter.onCallLook)
            except KeyboardInterrupt:
                print ("Interrupted by us ... Shutdown")
            t1.start()

            self.nao_rest()
            time.sleep(offset + duration + delay)

            # Counter to keep track of trial, blocks etc and sort out the warmup trials
            if(warmup and counter == 10) or (counter == 8 and not warmup):
                print("BLOCK " + str(trial_num) + " Complete")

                counter = 0
                trial_num = trial_num + 1
                warmup = False

                ## @TODO replace this with input to start the next trail, not just wait 5s:
                time.sleep(5)


            print(t)
            # Perform the movements based on the current trial:
            self.nao_move(t)

            # Wait some time, go to rest pose, wait again:
            time.sleep(0.5)
            self.nao_rest()
            time.sleep(1)

            counter = counter + 1

        print(len(list_block))
        
    # Generate all blocks
    ### Geneate a set of all 8 combinations (2x2x2), multiply by the total number of blocks in the trial, and add 2 for warmup to the first block
    def generate_all_block(self, warmup=True, num_blocks=1):

        # Generate full block:
        BD = [["L", "R"],["T", "V"] ,["C", "I"]]
        block = itertools.product(BD[0], BD[1], BD[2])

        # Convert to a list, should be a nicer way to do this?
        list_block = []
        for prod in block:
            list_block.append(tuple(prod))

        # Multiply the list by the number specified
        list_block = list_block * num_blocks

        # Randomize across all blocks:
        random.shuffle(list_block)
        # @TODO ramdomize in a way to not have more than 2(?) trials of the same type in a row.

        # Add the two trials at the start of the sequence for the warmup
        if warmup:
            list_block.insert(0, (random.choice(BD[0]), random.choice(BD[1]), random.choice(BD[2])))
            list_block.insert(0, (random.choice(BD[0]), random.choice(BD[1]), random.choice(BD[2])))

        return list_block
        
    # Nao Rest
    ## Send to neutral pose
    def nao_rest(self):
            self.CommandExecuter.tracker.lookAt([rest_position["x"], rest_position["y"], rest_position["z"]], 0, self.CommandExecuter.maxSpeed, False)
            self.CommandExecuter.pub_stimulus.publish(" , ")

    # Nao Move
    ## Send to the correct position, and send sequence to display to psychopy
    def nao_move(self, t):

        dir_to_look = (left_screen["x"], left_screen["y"], left_screen["z"])
        str_to_display = " , "

        # Check Left direction and congruency:
        if(t[0] == "L"):
            if(t[2] != "C"):
                dir_to_look = (right_screen["x"], right_screen["y"], right_screen["z"])
            # Check what value to display:
            if(t[1] == "T"):
                str_to_display = "t, "
            else:
                str_to_display = "v, "
        # Check Right direction and congruency:
        else:
            if(t[2] == "C"):
                dir_to_look = (right_screen["x"], right_screen["y"], right_screen["z"])
            # Check what value to display:
            if(t[1] == "T"):
                str_to_display = " ,t"
            else:
                str_to_display = " ,v"

        # update coordinates, and publish the stimuli
        self.CommandExecuter.updateCoordinates(dir_to_look[0], dir_to_look[1], dir_to_look[2] )
        self.CommandExecuter.pub_stimulus.publish(str_to_display)    

if __name__ == '__main__':

    #main()

    e = Experiment(NAO_IP, NAO_PORT)
    e.play_block(True, 2)