#!/usr/bin/env python
from psychopy import visual, core
from psychopy.hardware import keyboard
import configparser, re
from threading import Thread
from datetime import datetime
import rospy
from std_msgs.msg import String

class StimuliController:

    def __init__(self):
        
        ## Read the configurables from the config file
        self.config = configparser.ConfigParser()
        self.config.read('config.ini')

        # Convert config file string to tuple for the screen res
        self.screen_resolution = tuple(int(v) for v in re.findall("[0-9]+", self.config['Screen']['ScreenResolution']))
        self.full_screen = bool(self.config['Screen']['Fullscreen'])

        self.kb = keyboard.Keyboard()

        ## Set up an order for the conditions:
        self.conditions = [
                        [  ["", ""], ["", ">"], ["<", ""] ],
                        [  ["", ""], ["", ">"], ["<", ""] ]
                    ]


        ## Setup screen
        self.win0 = visual.Window(self.screen_resolution, screen=0, fullscr=self.full_screen)
        self.win1 = visual.Window(self.screen_resolution, screen=1, fullscr=self.full_screen)
        self.left_stimulus = ""
        self.right_stimulus = ""

        self.key_list = ['right', 'left']

        # ROS setup:
        rospy.init_node('psycopy', anonymous=True)
        rospy.Subscriber("stimulus", String, self.stimulusCb)
        self.pub_keypress = rospy.Publisher('keypress', String, queue_size=0)


        #self.preScreen()
        # Set the start time of the experiment
        self.time_start = datetime.now()

        ## Set up 2 threads - 1 for displaying the stimulus and the other for logging the keypresses:
        #display_thead = Thread(target=self.displayCondition, args=("left", "right", 5.0))
        #display_thead = Thread(target=self.runConditions, args=(self.conditions, 1.0))
        #display_thead = Thread(target=self.rosLoop)
        #keyhandler_thread = Thread(target=self.keyController)
        #keyhandler_thread.start()
        #display_thead.start()
        #display_thead.join()
        self.displayCondition('v','t')
        self.rosLoop()

    def __del__ (self):

        try:
            # Finish and close the windows
            print("Fin")
            self.win0.close()
            self.win0.close()
            core.quit()
        except Exception as e:
            print(e)

    def stimulusCb(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if data.data == "pre":
            self.left_stimulus = "screen one"
            self.right_stimulus = "screen two"
        else:
            try:
                s = data.data.split(",")
                self.left_stimulus = s[0]
                self.right_stimulus = s[1]
                self.kb.clock.reset()  # when you want to start the timer from
            except Exception as e:
                print(e)


    def rosLoop(self):

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.displayCondition(self.left_stimulus, self.right_stimulus)
            self.checkForKey(self.key_list)
            rate.sleep()

    ## Display a single condition:
    def displayCondition(self, left, right):

        if left and right != " ":
            print("Display Condition : ", left, right)

        msg0_msg = visual.TextStim(self.win0, text=left)
        msg1_msg = visual.TextStim(self.win1, text=right)
        msg0_msg.draw()
        msg1_msg.draw()
        self.win0.flip()
        self.win1.flip()   

    
    ## Loop to be threaded, to allow constant monitoring of key press
    def keyController(self):

        key_list = ['right', 'left']
        while True:
            self.waitforKeyPress(key_list)

    ## Blocking function to wait for keypress, also deals with quitting the application with 'q' or 'quit'
    def waitforKeyPress(self, key_list):

        key_list = key_list + ['quit', 'q']
        print("Waiting for keypress")

        self.kb.clock.reset()  # when you want to start the timer from
        keys = self.kb.getKeys(key_list, waitRelease=False)

        while len(keys) <= 0 :
            keys = self.kb.getKeys(key_list, waitRelease=False)
            if 'q' in keys:
                print("quits")
                core.quit()
            for key in keys:
                #print(key.name, key.rt, key.duration)
                print("Keypress: ", key.name, " @ ", datetime.now() - self.time_start)

        return keys

    def checkForKey(self, key_list):

        key_list = key_list + ['quit', 'q']
        print("Waiting for keypress")

        keys = self.kb.getKeys(key_list, waitRelease=False)
        print(keys)

        if 'q' in keys:
            print("quits")
            core.quit()
        if len(keys) > 0:
            for key in keys:
                self.pub_keypress.publish(key.name)


if __name__ == "__main__":
    sc = StimuliController()