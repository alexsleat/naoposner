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

        self.img_size = float(self.config['Screen']['ImgSize'])
        # The Y axis acts weight, because it's of a percentage of the total. So need the below factor applied to the height size:
        self.height_scalar = 0.35

        self.x_location = 0.8
        self.y_location = 0.2

        # Convert config file string to tuple for the screen res
        self.screen_resolution = tuple(int(v) for v in re.findall("[0-9]+", self.config['Screen']['ScreenResolution']))
        self.full_screen = bool(self.config['Screen']['Fullscreen'])

        self.kb = keyboard.Keyboard()

        ## Setup screen
        self.win0 = visual.Window(self.screen_resolution, screen=1, fullscr=self.full_screen)
        self.win1 = visual.Window(self.screen_resolution, screen=2, fullscr=self.full_screen)

        self.left_stimulus = ' '
        self.right_stimulus = ' '

        self.key_list = ['a', 'e']

        self.results_flag = False

        # ROS setup:
        rospy.init_node('psycopy', anonymous=True)
        rospy.Subscriber("stimulus", String, self.stimulusCb)
        rospy.Subscriber("results", String, self.resultsCb)

        self.pub_keypress = rospy.Publisher('keypress', String, queue_size=1)

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

    ## Callback when a results is sent from nao_cueing.py
    def resultsCb(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

        try:
            s = data.data.split(",")
            self.left_stimulus = s[0]
            self.right_stimulus = s[1]
            self.results_flag = True
            #self.kb.clock.reset()  # when you want to start the timer from
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
                self.results_flag = False
            except Exception as e:
                print(e)

    ## ROS main loop, display latest stimulus and check for keys
    def rosLoop(self):

        rate = rospy.Rate(120) # 120hz #run at double regular monitors refresh rate
        while not rospy.is_shutdown():

            if self.results_flag :
                self.displayResults(self.left_stimulus, self.right_stimulus)
            else:
                self.displayCondition(self.left_stimulus, self.right_stimulus, self.config['Screen']['TextOrImg'])
            self.checkForKey(self.key_list)
            rate.sleep()

    ## Display a single condition:
    def displayResults(self, left, right):

        # if left and right != " ":
        #     print("Display Results : ", left, right)

        # Draw a white rect for the background
        msg0_bg = visual.rect.Rect(self.win0, size=(800,600), fillColor='white')
        msg1_bg = visual.rect.Rect(self.win1, size=(800,600), fillColor='white')
        msg0_bg.draw()
        msg1_bg.draw()

        # Draw the text response, this will be overwritten if the img option is set:
        msg0_msg = visual.TextStim(self.win0, text=left)
        msg1_msg = visual.TextStim(self.win1, text=right)    

        # An almost-black text
        msg0_msg.colorSpace = 'rgb255'
        msg1_msg.colorSpace = 'rgb255'

        # Make it light green again
        msg0_msg.color = (0, 0, 0)  
        msg1_msg.color = (0, 0, 0)   

        msg0_msg.pos = (0, 0)
        msg1_msg.pos = (0, 0)

        # Draw and flip:
        msg0_msg.draw()
        msg1_msg.draw()
        self.win0.flip()
        self.win1.flip()   

    ## Display a single condition:
    def displayCondition(self, left, right, text_or_img="img"):

        # if left and right != " ":
        #     print("Display Condition : ", left, right)

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

                if left == "t" or left == "v":
                    left_img = "letter-" + str(left) + "-512.jpg"
                    #print("left img:: ", left_img)
                    msg0_msg = visual.ImageStim(self.win0, left_img) # set image
                elif left == "correct_t" or left == "correct_v":
                    left_img = "letter-" + str(left) + "-512.jpg"
                    #print("left img:: ", left_img)
                    msg0_msg = visual.ImageStim(self.win0, left_img) # set image
                elif left == "incorrect_t" or left == "incorrect_v":
                    left_img = "letter-" + str(left) + "-512.jpg"
                    #print("left img:: ", left_img)
                    msg0_msg = visual.ImageStim(self.win0, left_img) # set image                

            if right != ' ': 
                if right == "t" or right == "v":
                    right_img = "letter-" + str(right) + "-512.jpg"
                    #print("right img:: ", right_img)
                    msg1_msg = visual.ImageStim(self.win1, right_img) # set image
                elif right == "correct_t" or right == "correct_v":
                    right_img = "letter-" + str(right) + "-512.jpg"
                    #print("right img:: ", right_img)
                    msg1_msg = visual.ImageStim(self.win1, right_img) # set image
                elif right == "incorrect_t" or right == "incorrect_v":
                    right_img = "letter-" + str(right) + "-512.jpg"
                    #print("right img:: ", right_img)
                    msg1_msg = visual.ImageStim(self.win1, right_img) # set image          

        msg0_msg.size = (self.img_size, self.img_size * self.height_scalar)
        msg0_msg.pos = ((self.x_location * -1), self.y_location)
        msg1_msg.size = (self.img_size, self.img_size * self.height_scalar)
        msg1_msg.pos = (self.x_location , self.y_location)
        # Draw and flip:
        msg0_msg.draw()
        msg1_msg.draw()
        self.win0.flip()
        self.win1.flip()   

    ## Check key presses, quit if q
    def checkForKey(self, key_list):

        key_list = key_list + ['quit', 'q', 'c']
        keys = self.kb.getKeys(key_list, waitRelease=False)

        if 'q' in keys:
            key_logger =  "KeyPress,exit,exit,date"
            self.pub_keypress.publish(key_logger)
            print("quits")
            core.quit()
        if 'c' in keys:
            key_logger =  "KeyPress,ready,ready,date"
            self.pub_keypress.publish(key_logger)
            print("ready")
        if len(keys) > 0:
            print("RCVD keypress: ", keys)

            for key in keys:
                now = datetime.now()
                key_info = now.strftime("%d.%m.%y-%Hh%Mm%Ss%fns")

                kn = key.name
                kn = 't' if kn == 'a' else 'v'
                key_logger =  "KeyPress,{},{},{}".format(str(key.name), str(kn), str(key_info))
                self.pub_keypress.publish(key_logger)


if __name__ == "__main__":
    sc = StimuliController()