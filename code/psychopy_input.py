#!/usr/bin/env python
from psychopy import visual, core
from psychopy.hardware import keyboard
import configparser, re
from threading import Thread
from datetime import datetime
import rospy

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

        #self.preScreen()
        # Set the start time of the experiment
        self.time_start = datetime.now()

        ## Set up 2 threads - 1 for displaying the stimulus and the other for logging the keypresses:
        #display_thead = Thread(target=self.displayCondition, args=("left", "right", 5.0))
        display_thead = Thread(target=self.runConditions, args=(self.conditions, 1.0))
        keyhandler_thread = Thread(target=self.keyController)
        keyhandler_thread.start()
        display_thead.start()
        #display_thead.join()

        core.quit()

    def __del__ (self):

        try:
            # Finish and close the windows
            print("Fin")
            self.win0.close()
            self.win0.close()
        except Exception as e:
            print(e)

    ## Screen to confirm left and right are correct, can be used to make a pre-study screen. I-e keep both black until it begins
    def preScreen(self):
        ## Create a blank msg to clear the screen
        self.msg0_blank = visual.TextStim(self.win0, text=u"screen one")
        self.msg1_blank = visual.TextStim(self.win1, text=u"screen two")

        ## Draw it
        self.msg0_blank.draw()
        self.msg1_blank.draw()

        ## Flip seems to make it non blocking on the screen? need to look it up.
        self.win0.flip()
        self.win1.flip()

        ## wait 2 seconds
        #core.wait(2)

        self.waitforKeyPress(['right', 'left'])

    ## Display a single condition:
    def displayCondition(self, left, right, time):

        print("Display Condition : ", left, right, time)

        msg0_msg = visual.TextStim(self.win0, text=left)
        msg1_msg = visual.TextStim(self.win1, text=right)
        msg0_msg.draw()
        msg1_msg.draw()
        self.win0.flip()
        self.win1.flip()   

        # set a clock and wait for a second
        clock = core.Clock()
        while clock.getTime() < time:  # Clock times are in seconds
            pass
    
    ## Loop to be threaded, to allow constant monitoring of key press
    def keyController(self):

        key_list = ['right', 'left']
        while True:
            self.waitforKeyPress(key_list)

    ## Run through all the conditions in the list:
    def runConditions(self, conditions, time):

        # loop conditions:
        for x in range(len(conditions)):
            print("x:", x)
            for y in range(len(conditions[x])):
                print("y:", y)

                # Create msg, draw and flip @TODO function this up for each screen
                msg0_msg = visual.TextStim(self.win0, text=conditions[x][y][0])
                msg1_msg = visual.TextStim(self.win1, text=conditions[x][y][1])
                msg0_msg.draw()
                msg1_msg.draw()
                self.win0.flip()
                self.win1.flip()     

                # set a clock and wait for a second
                print("Condition: ", conditions[x][y], " @ ", datetime.now() - self.time_start)
                clock = core.Clock()
                while clock.getTime() < time:  # Clock times are in seconds
                    pass

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


if __name__ == "__main__":
    sc = StimuliController()