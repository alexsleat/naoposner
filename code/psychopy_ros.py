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

        ## Setup screen
        self.win0 = visual.Window(self.screen_resolution, screen=0, fullscr=self.full_screen)
        self.win1 = visual.Window(self.screen_resolution, screen=1, fullscr=self.full_screen)

        self.left_stimulus = ' '
        self.right_stimulus = ' '

        self.key_list = ['right', 'left']

        # ROS setup:
        rospy.init_node('psycopy', anonymous=True)
        rospy.Subscriber("stimulus", String, self.stimulusCb)
        self.pub_keypress = rospy.Publisher('keypress', String, queue_size=0)

        #self.preScreen()
        # Set the start time of the experiment
        self.time_start = datetime.now()

        self.displayCondition(' ',' ')
        self.rosLoop()

    ## Destructors: "i felt like destroying something beautiful"
    def __del__ (self):

        try:
            # Finish and close the windows
            print("Fin")
            self.win0.close()
            self.win0.close()
            core.quit()
        except Exception as e:
            print(e)

    ## Callback when a stimulus is sent from nao_cueing.py
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

    ## ROS main loop, display latest stimulus and check for keys
    def rosLoop(self):

        rate = rospy.Rate(120) # 120hz #run at double regular monitors refresh rate
        while not rospy.is_shutdown():
            self.displayCondition(self.left_stimulus, self.right_stimulus, self.config['Screen']['TextOrImg'])
            self.checkForKey(self.key_list)
            rate.sleep()

    ## Display a single condition:
    def displayCondition(self, left, right, text_or_img="img"):

        if left and right != " ":
            print("Display Condition : ", left, right)

        # Draw a white rect for the background
        msg0_bg = visual.rect.Rect(self.win0, size=(800,600), fillColor='white')
        msg1_bg = visual.rect.Rect(self.win1, size=(800,600), fillColor='white')
        msg0_bg.draw()
        msg1_bg.draw()

        # Draw the text response, this will be overwritten if the img option is set:
        msg0_msg = visual.TextStim(self.win0, text=left)
        msg1_msg = visual.TextStim(self.win1, text=right)

        # If the img option is set, load the images based on the stimulus provided:
        if text_or_img == "img":
            if left != ' ': 
                left_img = "letter-" + str(left) + "-512.jpg"
                print("left img:: ", left_img)
                msg0_msg = visual.ImageStim(self.win0, left_img) # set image
            if right != ' ': 
                right_img = "letter-" + str(right) + "-512.jpg"
                msg1_msg = visual.ImageStim(self.win1, right_img) # set image            

        # Draw and flip:
        msg0_msg.draw()
        msg1_msg.draw()
        self.win0.flip()
        self.win1.flip()   

    ## Check key presses, quit if q
    def checkForKey(self, key_list):

        key_list = key_list + ['quit', 'q']
        keys = self.kb.getKeys(key_list, waitRelease=False)

        if 'q' in keys:
            print("quits")
            core.quit()
        if len(keys) > 0:
            print("RCVD keypress: ", keys)
            for key in keys:
                self.pub_keypress.publish(key.name)


if __name__ == "__main__":
    sc = StimuliController()