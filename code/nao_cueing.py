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
left_screen = {"x":1.5, "y":1, "z":0}
right_screen = {"x":1.5, "y":-1, "z":0}
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
        self.resting = True

        self.reaction_times = []

        global config 
        self.participant_key = str(config['Timing']['ParticipantKey'])

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
        self.posture.goToPosture("Crouch", 1.0)
        time.sleep(3)

        self.dt_comand_key = 42

        # ROS Pubs n subs:
        self.pub_stimulus = rospy.Publisher('stimulus', String, queue_size=0)
        self.pub_logger = rospy.Publisher('logger', String, queue_size=0)

        self.sub_keypress = rospy.Subscriber('keypress', String, self.keypressCb)
        rospy.init_node('nao_cueing', anonymous=True)

    def wait_for_keypress_or_2s(self):
        """
            This function will process the given timestamp of when NAO turned the head.
            Wait time can be adopted by changing the loop boundaries.
            Currently its 100*0.02s = 2s

            in:
            - Naos timestamp in datetime format

            return: None
            Function will wait for 2s or until key is pressed.            
        """

        i = 0

        # Loop is idle for 2s waiting for participants to press key
        # @Alex maybe we should run this in a thread as well ?

        while(i<100):

            if self.dt_comand_key < 2:
                self.valid_trial = True
                self.dt_comand_key = 42
                return None
            else:
                time.sleep(0.02)
                i += 1

        self.valid_trial = False
        return None

    def keypressCb(self, data):
        print(data.data)
        # @TODO add logging here for key presss

        # Parse the message string
        key_press = data.data.split(',')

        # Store the key press data in member variables
        self.last_key_timestamp = key_press[2]
        self.key_name = key_press[1]

        dt1 = datetime.strptime(self.head_info2, '%d.%m.%y-%Hh%Mm%Ss%fns')
        dt2 = datetime.strptime(self.last_key_timestamp, '%d.%m.%y-%Hh%Mm%Ss%fns')

        # @TODO only do this when the key is pressed participants should press
        # Replace the True with self.key_name == participants_key
        self.dt_comand_key = (dt2 - dt1).total_seconds()
        # if self.key_name in self.participant_key:
        #     self.reaction_times.append(self.dt_comand_key)
        self.reaction_times.append(self.dt_comand_key)

    def updateCoordinates(self,x,y,z):
        """Function that simply updates Parameters"""
        self.x = x
        self.y = y
        self.z = z
        self.resting = False

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
                self.head_info2 = now.strftime("%d.%m.%y-%Hh%Mm%Ss%fns")
                head_logger =  "HeadTurn,{},{}".format(str(head_info1), str(self.head_info2))
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
        self.corr_trials = 0
        self.num_trials = 0

        # Reading variables from the config file wrt. Timing
        global config # To use the gloabl config parser
        # Tiem it takes for NAO to turn its head to make the complete 1000 ms ITTI
        self.timing_for_head_turn = int(config['Timing']['NaoHeadTurn'])

    # Play block:
    ## Alternative method to Main:
    def play_block(self, warmup = True, num_blocks = 5):


        # # At the beginning of the loop start the thread
        try:
            # Define a thread parallel to the main one
            # Thread runs the function that controlls NAOs head
            t1 = threading.Timer(offset, self.CommandExecuter.onCallLook)
        except KeyboardInterrupt:
            print ("Interrupted by us ... Shutdown")

        self.CommandExecuter.dt_comand_key = 42
        t1.start()
        time.sleep(offset + duration + delay)

        self.CommandExecuter.pub_stimulus.publish(" , ")
        time.sleep(1)

        # Generate & randomize the full block:
        list_block, block_types = self.generate_all_block(warmup, num_blocks)

        # Run through a full trial:
        counter = 0
        block_num = 1

        self.participant_interface("Block")

        for t in (list_block):
            # get NAO to start in resting position
            # if not self.CommandExecuter.resting:
            #     self.nao_rest()
            # self.nao_rest()
            start_time = self.get_formatted_datetime()
            self.participant_interface("Trial")
            # # At the beginning of the loop start the thread
            # try:
            #     # Define a thread parallel to the main one
            #     # Thread runs the function that controlls NAOs head
            #     t1 = threading.Timer(offset, self.CommandExecuter.onCallLook)
            # except KeyboardInterrupt:
            #     print ("Interrupted by us ... Shutdown")
            # t1.start()
            # time.sleep(offset + duration + delay)

            # Counter to keep track of trial, blocks etc and sort out the warmup trials
            if(warmup and counter == 10) or (counter == 8 and not warmup):
                print("BLOCK " + str(block_num) + " Complete")

                counter = 0
                block_num = block_num + 1
                warmup = False

                self.participant_interface("Block")
                ## @TODO replace this with input to start the next trail, not just wait 5s:


            # Perform the movements based on the current trial:
            self.nao_move(t)

            # Wait some time, go to rest pose, wait again:
            self.CommandExecuter.wait_for_keypress_or_2s()
            time.sleep(1-self.timing_for_head_turn)
            self.nao_rest()
            

            end_time = self.get_formatted_datetime()
            #print(t, start_time, end_time, self.get_trial_type(t[1], t[0], t[2]), self.trial_number, block_types[block_num-1], self.PID, self.AGE)
            valid_trial = self.CommandExecuter.valid_trial
            if valid_trial:
                self.corr_trials += 1
            # Information to be published:
            # Participants ID (PID), Participants Age (AGE), Start and end time of trial, block type, trial type, trial_number (Consecutive increasing each block), validity(True/False)
            information_to_publish = [self.PID, self.AGE, start_time, end_time, self.block_type, self.get_trial_type(t[1], t[0], t[2]), self.trial_number, valid_trial]
            log_string = str(t[0])+","+str(t[1])+","+str(t[2])+","
            for info in information_to_publish:
                log_string += (str(info)+",")
            #print(log_string)
            self.CommandExecuter.pub_logger.publish(log_string)

            self.trial_number += 1
            counter = counter + 1
            self.num_trials += 1

        # if not self.CommandExecuter.resting:
        #     self.nao_rest()
        self.nao_rest()

        time.sleep(0.5)
        print("LAST BLOCK COMPELTE total number of trials for this participant: {}".format(len(list_block)))
        reaction_times = np.array([self.CommandExecuter.reaction_times])
        print("Mean reaction times: {}".format(np.round(np.mean(reaction_times), 2)))
        print("{} Perc. of Trials were valid".format(np.round((float(self.corr_trials)/float(self.num_trials)), 4)*100))
        self.CommandExecuter.alive = False

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

        # Create list of possible block types
        block_type = "N/A"
        block_types= num_blocks*[block_type]

        return list_block, block_types

    # Nao Rest
    ## Send to neutral pose
    def nao_rest(self):
            # self.CommandExecuter.tracker.lookAt([rest_position["x"], rest_position["y"], rest_position["z"]], 0, self.CommandExecuter.maxSpeed, False)
            self.CommandExecuter.updateCoordinates(rest_position["x"], rest_position["y"], rest_position["z"])
            self.CommandExecuter.pub_stimulus.publish(" , ")
            self.CommandExecuter.resting = True

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

    # Function to wait for user input -> Signalling readiness
    def participant_interface(self, ready_for = None):

        if ready_for == "Block":
            user_input = raw_input("Press 'o' for occluded block, b for baseline:\t")

            if (user_input == 'b') or (user_input == 'o'):
                self.block_type = user_input

                print("Ready for next block, proceed with next block")
                self.nao_rest()

                self.CommandExecuter.flash_eyes("green")
                return
            else:
                print("You are not ready, please try again")
                self.participant_interface(ready_for)

        if ready_for == "Trial":
            self.CommandExecuter.flash_eyes("blue")
            return

    def get_formatted_datetime(self):
        # Method if we want to do the timing over rospy
        # now = rospy.Time.now()
        # secs = now.secs
        # nsecs = now.nsecs
        # dt = datetime.fromtimestamp(secs)
        # dt_str = dt.strftime("%d.%m.%y-%Hh%Mm%Ss")
        # return dt_str + str(int(nsecs/1000000)).zfill(6)

        now = datetime.now()
        return now.strftime("%d.%m.%y-%Hh%Mm%Ss%fns")

    def get_trial_type(self, letter, gaze_direction, congruency):
        # Create a dictionary that maps the combination of letter, gaze_direction, and congruency to an integer
        trial_type_map = {
            "TLC": 1,
            "TLI": 2,
            "TRC": 3,
            "TRI": 4,
            "VLC": 5,
            "VLI": 6,
            "VRC": 7,
            "VRI": 8
                }
        # Concatenate the input variables to create the key for the dictionary
        key = letter + gaze_direction + congruency
        # Return the integer value associated with the key
        return trial_type_map[key]

    def create_new_aprticipant(self):
        self.PID = raw_input("Experimenter: Enter the participants ID (PID)")
        self.AGE = raw_input("Experimenter: Enter the participants age(AGE)")

if __name__ == '__main__':
    # # Start a ROS node that will run in parallel to the main loop
    # rospy.init_node('my_node')

    while not rospy.is_shutdown():

        #main()
        e = NaoPosnerExperiment(NAO_IP, NAO_PORT)
        e.create_new_aprticipant()
        e.nao_rest()
        time.sleep(2)

        # # Create a new parallel thread for the ROS subscription
        # def ros_thread():
        #     rospy.Subscriber('keypress', String, e.key_callback)
        #     rospy.spin()

        # thread = threading.Thread(target=ros_thread)
        # thread.start()

        # Run the main loop with the blocks and trials
        e.play_block(True, 2)

        # try:
        #     e.play_block(True, 2)
        # except Exception as expt:
        #     print(expt)
        #     e.CommandExecuter.alive = False
        # e.nao_rest()

        # # Wait until ending the programm, before the ROS thread finished
        # thread.join()

    # # Clean up
    # e.nao_rest()
    # rospy.signal_shutdown('Done')

    e.myBroker.shutdown()
    sys.exit(0)
