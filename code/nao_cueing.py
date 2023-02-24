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

import csv
import os.path

now = datetime.now()
date_time_str = now.strftime("%d.%m.%y-%Hh%Mm%Ss")
CSV_FILENAME = "logs/" + date_time_str + '.csv'

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
offset = float(config['Timing']['StimuliOffset'])
stimuli_wait = float(config['Timing']['StimuliWait'])
key_wait = float(config['Timing']['KeyWait'])
key_total = float(config['Timing']['KeyTotal'])

# That is the duration of how long it takes to rotate NAOS head
duration = float(config['Timing']['NaoHeadDuration'])
# Delay determines how much ms are between head movement and stimuli presentation
delay = float(config['Timing']['HeadStimuliDelay'])

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
        self.maxSpeed = float(config['Timing']['NaoMaxSpeed'])

        self.x = float(config['Experiment']['X'])
        self.y = float(config['Experiment']['Y'])
        self.z = float(config['Experiment']['Z'])

        self.old_x = 0.0
        self.old_y = 0.0
        self.old_z = 0.0

        self.alive = True
        self.resting = True

        self.reaction_times = []

        self.exit_flag = False

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
        self.last_key_pressed = -1
        self.last_key_timestamp = -1
        self.keypress_counter = 0
        self.participant_ready = False

        # ROS Pubs n subs:
        self.pub_stimulus = rospy.Publisher('stimulus', String, queue_size=0)
        self.pub_logger = rospy.Publisher('logger', String, queue_size=0)
        self.pub_result = rospy.Publisher('results', String, queue_size=0)

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

        # Clear the key pressed and timestamp:
        self.last_key_pressed = -1
        self.last_key_timestamp = -1

        # response_flag = False

        self.dt_comand_key = 42
        keypress_start_time = datetime.now()

        while(i < (key_total/key_wait)):

            # if self.dt_comand_key < 2 and response_flag:
            if self.dt_comand_key < 2:
                self.valid_trial = True
                self.reaction_times.append(self.dt_comand_key)
                response_flag = True
                self.dt_comand_key = 42
                return i
            else:
                time.sleep(key_wait)
                i += 1

        keypress_end_time = datetime.now()
        elapsed_time = (keypress_start_time-keypress_end_time).microseconds
        self.valid_trial = False
        self.reaction_times.append(key_total)
        return None

    def keypressCb(self, data):
        # print("KeyPressed: ", data.data)
        # Parse the message string
        key_press = data.data.split(',')

        if(key_press[1] == "exit"):
            self.exit_flag
            return -1
        elif(key_press[1] == "ready"):
            self.participant_ready = True
            return 1

        self.keypress_counter = self.keypress_counter + 1

        # Store the key press data in member variables
        self.last_key_timestamp = key_press[3]
        self.last_key_pressed = key_press[2]

        if(self.head_info2 != ""):
            dt1 = datetime.strptime(self.head_info2, '%d.%m.%y-%Hh%Mm%Ss%fns')
            dt2 = datetime.strptime(self.last_key_timestamp, '%d.%m.%y-%Hh%Mm%Ss%fns')

            # Store time of key press
            self.dt_comand_key = (dt2 - dt1).total_seconds()

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

                #print("Head turning: {},{},{}".format(str(self.x), str(self.y), str(self.z)))

                self.useWholeBody = False
                self.frame = 0 #0 - TORSO, 1 - World, 2- Robot
                now = datetime.now()
                head_info1 = now.strftime("%d.%m.%y-%Hh%Mm%Ss%fns")
                # head_logger = "Head turned at: {}".format(str(head_info))
                # self.pub_logger.publish(head_logger)

                try:
                    self.tracker.lookAt([self.x, self.y, self.z], self.frame, self.maxSpeed, self.useWholeBody)
                except Exception as excpt:
                    print(excpt)
                    
                now = datetime.now()
                self.head_info2 = now.strftime("%d.%m.%y-%Hh%Mm%Ss%fns")
                head_logger =  "HeadTurn,{},{}".format(str(head_info1), str(self.head_info2))
                self.pub_logger.publish(head_logger)

                # Also log to CSV file:
                with open(CSV_FILENAME, 'a') as f:
                    f.write(head_logger + "\n")

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

        if NAO_FLAG:

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
        self.head_turn_sent_time = ""
        self.stimulus_sent_time = ""

        headers_string = "PID,Age,Trial_Start,Trial_End,Block_Type,LeftRight,TV,Congruency,Trial_Type,Trial_No,Trial_Valid,Letter_Displayed,User_KeyPress,User_KeyTimeStamp,User_KeyCounter,Onset_HeadMovement,Onset_Stimulus"

        with open(CSV_FILENAME, 'a') as f:
            f.write(headers_string + "\n")

        # Reading variables from the config file wrt. Timing
        global config # To use the gloabl config parser
        # Tiem it takes for NAO to turn its head to make the complete 1000 ms ITTI
        self.timing_for_head_turn = int(config['Timing']['NaoHeadTurn'])

    # Play block:
    ## Alternative method to Main:
    def play_block(self, warmup = True, num_trials = 8, block_size = 8, trials_before_feedback=8, training=False):


        # # At the beginning of the loop start the thread
        # try:
        #     # Define a thread parallel to the main one
        #     # Thread runs the function that controlls NAOs head
        #     t1 = threading.Timer(offset, self.CommandExecuter.onCallLook)
        # except KeyboardInterrupt:
        #     print ("Interrupted by us ... Shutdown")

        self.CommandExecuter.dt_comand_key = 42
        # t1.start()
        time.sleep(offset + duration + delay)
        self.CommandExecuter.pub_stimulus.publish(" , ")

        time.sleep(1)

        # Generate & randomize the full block:
        #print(warmup, num_trials/8, trials_before_feedback, num_trials/trials_before_feedback)
        list_block, block_types = self.generate_all_block(warmup, num_trials/8, trials_before_feedback, num_trials/trials_before_feedback)

        # Run through a full trial:
        counter = 0
        correct_counter = 0
        block_num = 1

        self.participant_interface("Block")

        self.CommandExecuter.participant_ready = False
        while not self.CommandExecuter.participant_ready:
            pass
        self.CommandExecuter.participant_ready = False
        time.sleep(1)

        for t in (list_block):

            # Get the letter displayed in the trail (in lower case):
            letter_displayed_in_trial = t[1]
            letter_displayed_in_trial = 't' if letter_displayed_in_trial == 'T' else 'v'
            user_pressed = str_to_display = ''
            self.CommandExecuter.last_key_pressed = -1

            self.CommandExecuter.keypress_counter = 0

            self.CommandExecuter.pub_stimulus.publish(" , ")

            # Counter to keep track of trial, blocks etc and sort out the warmup trials
            if(warmup and counter == (trials_before_feedback + 2)) or (counter == trials_before_feedback and not warmup):
                print("BLOCK " + str(block_num) + " Complete")

                counter = 0
                correct_counter = 0
                block_num = block_num + 1
                warmup = True

                self.CommandExecuter.participant_ready = False

                if(block_num -1 == (num_trials/trials_before_feedback)/2):
                    self.participant_interface("Block")

                    while not self.CommandExecuter.participant_ready:
                        pass
                    self.CommandExecuter.participant_ready = False
                    time.sleep(1)
                else:
                    while not self.CommandExecuter.participant_ready:
                        pass
                    self.CommandExecuter.participant_ready = False
                    time.sleep(1)
                ## @TODO replace this with input to start the next trail, not just wait 5s:

            self.CommandExecuter.set_eyes(True)

            # Record the start time of the trail:
            start_time = self.get_formatted_datetime()
            self.participant_interface("Trial")

            # Perform the movements based on the current trial:
            self.nao_move(t)
            self.CommandExecuter.set_eyes(False)

            # Wait some time, go to rest pose, wait again:
            #print("Sleep 6")
            sleep_time = self.CommandExecuter.wait_for_keypress_or_2s()


            ## In the training round, show the in red or green when they press the key to show they're correct:
                # But we always want to calculate the correct counter for after block display
            # Get the key the user pressed:
            user_pressed = self.CommandExecuter.last_key_pressed 

            # Generate the response msg for the letter displayed on the screen
            if letter_displayed_in_trial == user_pressed:
                correct_counter = correct_counter + 1

            if(training):

                # Generate the response msg for the letter displayed on the screen
                if letter_displayed_in_trial == user_pressed:
                    str_to_display = "correct_"  
                else:
                    str_to_display = "incorrect_"
                str_to_display = str_to_display + letter_displayed_in_trial

                # Display it on the correct side:
                if(t[0] == "L"):
                    str_to_display = str_to_display + ", "
                else:
                    str_to_display = " ," + str_to_display
                self.CommandExecuter.pub_stimulus.publish(str_to_display)
            else:
                self.CommandExecuter.pub_stimulus.publish(" , ")

            if(sleep_time != None):
                time_remaining = (key_total / key_wait) - sleep_time
                #print("Time remaining, ", time_remaining)
                for i in range(int(time_remaining)):
                    time.sleep(key_wait)

            #print("Sleep 1")
            #time.sleep(1)
            self.nao_rest()
            #print("Sleep 2")
            time.sleep(self.timing_for_head_turn)

            ###########################
            #### Log the details:
            end_time = self.get_formatted_datetime()
            #print(t, start_time, end_time, self.get_trial_type(t[1], t[0], t[2]), self.trial_number, block_types[block_num-1], self.PID, self.AGE)
            valid_trial = self.CommandExecuter.valid_trial
            if valid_trial:
                self.corr_trials += 1
            # Information to be published:
            # Participants ID (PID), Participants Age (AGE), Start and end time of trial, block type, trial type, trial_number (Consecutive increasing each block), validity(True/False), letter_displayed_in_trial, user pressed key, user pressed key timestamp
            information_to_publish = [  self.PID, 
                                        self.AGE, 
                                        start_time, 
                                        end_time, 
                                        self.block_type, 
                                        str(t[0]),
                                        str(t[1]),
                                        str(t[2]),
                                        self.get_trial_type(t[1], t[0], t[2]), 
                                        self.trial_number, 
                                        valid_trial, 
                                        letter_displayed_in_trial, 
                                        self.CommandExecuter.last_key_pressed, 
                                        self.CommandExecuter.last_key_timestamp,
                                        self.CommandExecuter.keypress_counter,
                                        self.head_turn_sent_time,
                                        self.stimulus_sent_time
                                        ]

            # log_string = str(t[0])+","+str(t[1])+","+str(t[2])+","
            log_string = ""
            for info in information_to_publish:
                log_string += (str(info)+",")
            #print(log_string)logger
            self.CommandExecuter.pub_logger.publish(log_string)

            # Also log to CSV file:
            with open(CSV_FILENAME, 'a') as f:
                f.write(log_string + "\n")

            self.trial_number += 1
            counter = counter + 1
            self.num_trials += 1

            # During a training block, show the results of each trial after for ResultTimer seconds.
            reaction_times = -1
            if(training):
                reaction_times = self.CommandExecuter.reaction_times[counter-1]

                ### Display on the same side as letter:
                if(t[0] == "L"):
                    self.CommandExecuter.pub_result.publish("Response Time: {}, ".format(str(np.round(reaction_times, 2))))
                else:
                    self.CommandExecuter.pub_result.publish(" ,Response Time: {}".format(str(np.round(reaction_times, 2))))

                # Display on btoh:
                # self.CommandExecuter.pub_result.publish("Response Time: {},{}/{} Correct".format(str(np.round(reaction_times, 2)), str(correct_counter), str(counter)))
                #print("Sleep 3")
                time.sleep(int(config['Timing']['ResultTimer']))

            if ((counter == trials_before_feedback) and not warmup) or (counter == (trials_before_feedback + 2) and warmup):
                # Print results to screen:
                print("Counter", counter, " trials_before_feedback", trials_before_feedback)
                reaction_times = self.CommandExecuter.reaction_times
                self.CommandExecuter.pub_result.publish("Average Response: {}s,{}/{} Correct".format(str(np.round(np.mean(reaction_times), 2)), str(correct_counter), str(counter)))
                #print("Sleep 4")
                time.sleep(int(config['Timing']['ResultTimer']))
                correct_counter = 0


            if self.CommandExecuter.exit_flag:
                return 1

        # if not self.CommandExecuter.resting:
        #     self.nao_rest()
        self.CommandExecuter.pub_stimulus.publish(" , ")
        self.nao_rest()

        
        print("LAST BLOCK COMPELTE total number of trials for this participant: {}".format(len(list_block)))
        reaction_times = np.array([self.CommandExecuter.reaction_times])
        print("Mean reaction times: {}".format(np.round(np.mean(reaction_times), 2)))
        print("{} Perc. of Trials were valid".format(np.round((float(self.corr_trials)/float(self.num_trials)), 4)*100))

        #print("Sleep 5")
        time.sleep(0.5)

    # Generate all blocks
    ### Geneate a set of all 8 combinations (2x2x2), multiply by the total number of blocks in the trial, and add 2 for warmup to the first block
    def generate_all_block(self, warmup=True, num_chunks=1, block_length=8, number_of_blocks=1):

        print("Generating Blocks ", num_chunks)

        # Generate full block:
        BD = [["L", "R"],["T", "V"] ,["C", "I"]]
        block = itertools.product(BD[0], BD[1], BD[2])

        # Convert to a list, should be a nicer way to do this?
        list_block = []
        calculated_chunk_size = 0
        for prod in block:
            list_block.append(tuple(prod))
            calculated_chunk_size = calculated_chunk_size + 1

        print(calculated_chunk_size)

        # Multiply the list by the number specified
        list_block = list_block * num_chunks

        # Randomize across all blocks:
        random.shuffle(list_block)
        # @TODO ramdomize in a way to not have more than 2(?) trials of the same type in a row.

        repeaters = 1
        while repeaters  > 0:

            repeaters = 0

            random.shuffle(list_block)

            for i in range(len(list_block) - 2):
                    cd = list_block[i][0]
                    fd1 = list_block[i+1][0]
                    fd2 = list_block[i+2][0]

                    cl = list_block[i][1]
                    fl1 = list_block[i+1][1]
                    fl2 = list_block[i+2][1]

                    if (cd == fd1) and (cd == fd2) and (cl == fl1) and (cl == fl2):
                        #print(cd, fd1, fd2)
                        repeaters = repeaters + 1
                    else:
                        pass

        print("Origi: ", list_block, len(list_block))
        print("Repeaters: ", repeaters)


        ## This adds warm ups to each block:
        blocks_with_warmup = []
        block_counter = 0
        feedback_length = (num_chunks * calculated_chunk_size) / block_length
        print("feedback len", feedback_length)
        for b in range(number_of_blocks):

            # # Add the two trials at the start of the sequence for the warmup
            if warmup:
                blocks_with_warmup.append((random.choice(BD[0]), random.choice(BD[1]), random.choice(BD[2])))
                blocks_with_warmup.append((random.choice(BD[0]), random.choice(BD[1]), random.choice(BD[2])))
                # blocks_with_warmup.append(('Z', 'Z', 'Z'))
                # blocks_with_warmup.append(('Z', 'Z', 'Z'))

            for c in range(block_length/calculated_chunk_size):
                for t in range(calculated_chunk_size):
                    blocks_with_warmup.append(list_block[block_counter])
                    block_counter = block_counter + 1
        list_block = blocks_with_warmup



        # # Add the two trials at the start of the sequence for the warmup
        # if warmup:
        #     list_block.append((random.choice(BD[0]), random.choice(BD[1]), random.choice(BD[2])))
        #     list_block.append((random.choice(BD[0]), random.choice(BD[1]), random.choice(BD[2])))
        #     # blocks_with_warmup.append(('Z', 'Z', 'Z'))
        #     # blocks_with_warmup.append(('Z', 'Z', 'Z'))


        print("Blocks with WARM", list_block, len(list_block))

        # Create list of possible block types
        block_type = "N/A"
        block_types= num_chunks*[block_type]

        #print("Total number of trials; ", len(list_block))

        return list_block, block_types

    # Nao Rest
    ## Send to neutral pose
    def nao_rest(self):

        if NAO_FLAG:
            self.CommandExecuter.tracker.lookAt([rest_position["x"], rest_position["y"], rest_position["z"]], 0, self.CommandExecuter.maxSpeed, False)
            now = datetime.now()
            self.CommandExecuter.head_info2 = now.strftime("%d.%m.%y-%Hh%Mm%Ss%fns")
            #self.CommandExecuter.updateCoordinates(rest_position["x"], rest_position["y"], rest_position["z"])
            self.CommandExecuter.pub_stimulus.publish(" , ")
            self.CommandExecuter.resting = True

    # Nao Move
    ## Send to the correct position, and send sequence to display to psychopy
    def nao_move(self, t):

        dir_to_look = (right_screen["x"], right_screen["y"], right_screen["z"])
        str_to_display = "N/A"

        #print("Nao Move", t)

        # Check Left direction and congruency:
        if(t[0] == "L"):
            if(t[2] != "C"):
                dir_to_look = (left_screen["x"], left_screen["y"], left_screen["z"])
            # Check what value to display:
            if(t[1] == "T"):
                str_to_display = "t, "
            elif (t[1] == "V"):
                str_to_display = "v, "
        # Check Right direction and congruency:
        elif (t[0] == "R"):
            if(t[2] == "C"):
                dir_to_look = (left_screen["x"], left_screen["y"], left_screen["z"])
            # Check what value to display:
            if(t[1] == "T"):
                str_to_display = " ,t"
            elif (t[1] == "V"):
                str_to_display = " ,v"

        # nao_head_start_time = datetime.now()
        if NAO_FLAG:
            # update coordinates, and publish the stimuli
            #self.CommandExecuter.updateCoordinates(dir_to_look[0], dir_to_look[1], dir_to_look[2] )
            # self.CommandExecuter.tracker.lookAt([dir_to_look[0], dir_to_look[1], dir_to_look[2]], 0, self.CommandExecuter.maxSpeed, False)
            self.CommandExecuter.tracker.lookAt([dir_to_look[0], dir_to_look[1], dir_to_look[2]], 0, self.CommandExecuter.maxSpeed, False)
            now = datetime.now()
            self.head_turn_sent_time = now.strftime("%d.%m.%y-%Hh%Mm%Ss%fns")
            self.CommandExecuter.head_info2 = now.strftime("%d.%m.%y-%Hh%Mm%Ss%fns")

        # nao_head_end_time = datetime.now()
        # elapsed_time = (nao_head_start_time-nao_head_end_time).microseconds
        #time.sleep(stimuli_wait)
        now = datetime.now()
        self.stimulus_sent_time = now.strftime("%d.%m.%y-%Hh%Mm%Ss%fns")
        self.CommandExecuter.pub_stimulus.publish(str_to_display)

        # print("Time to perform command: ", elapsed_time)

    # Function to wait for user input -> Signalling readiness
    def participant_interface(self, ready_for = None):

        if ready_for == "Block":
            user_input = raw_input("Press 'o' for occluded block, b for baseline : ")

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
        print("#############################")
        print("#############################")
        print("")
        self.PID = raw_input("Experimenter: Enter the participants ID (PID) : ")
        self.AGE = raw_input("Experimenter: Enter the participants age(AGE) : ")

if __name__ == '__main__':
    # # Start a ROS node that will run in parallel to the main loop
    # rospy.init_node('my_node')

    e = NaoPosnerExperiment(NAO_IP, NAO_PORT)

    while not e.CommandExecuter.exit_flag:

        try: 
            e.create_new_aprticipant()
            e.nao_rest()

            # # Run the main loop with the blocks and trials
            number_of_trials = int(config['Experiment']['TotalNumberOfTrials'])
            size_of_block = int(config['Experiment']['BlockSize'])

            training_block = bool(config['Experiment']['TrainingBlock'])
            training_size = int(config['Experiment']['TrainingSize'])

            
            # Training Block
            if(training_block):
               e.play_block(training_block, training_size, training_size, training_size, True)

            e.play_block(True, number_of_trials, 8, size_of_block, False)            

            ### def generate_all_block(self, warmup=True, num_chunks=1, block_length=8):
            ## #print(True, number_of_trials/8, size_of_block, number_of_trials/size_of_block)
            #e.generate_all_block(False, 3)
            # e.generate_all_block(True, number_of_trials/8, size_of_block, number_of_trials/size_of_block)
            sys.exit(0)
            
        except Exception as expt:
            e.CommandExecuter.alive = False
            print(expt)
            e.myBroker.shutdown()
            sys.exit(0)

    # # Clean up
    # e.nao_rest()
    # rospy.signal_shutdown('Done')
    e.myBroker.shutdown()
    sys.exit(0)
