#!/usr/bin/env python
from psychopy import visual, core

#SCREEN_SIZE = (1920, 1080)      # 1080p 
SCREEN_SIZE = (2560, 1440)      # 1440p
#SCREEN_SIZE = (1368, 768)      # 768p

## Setup screen
win0 = visual.Window(SCREEN_SIZE, screen=0, fullscr=True)
win1 = visual.Window(SCREEN_SIZE, screen=1, fullscr=True)

## Create a blank msg to clear the screen
msg0_blank = visual.TextStim(win0, text=u"screen one")
msg1_blank = visual.TextStim(win1, text=u"screen two")

## Draw it
msg0_blank.draw()
msg1_blank.draw()

## Flip seems to make it non blocking on the screen? need to look it up.
win0.flip()
win1.flip()

## wait 2 seconds
core.wait(2)

## Set up an order for the conditions:
conditions = [
                [  ["", ""], ["", ">"], ["<", ""] ],
                [  ["", ""], ["", ">"], ["<", ""] ]
            ]

# loop conditions:
for x in range(len(conditions)):
    print("x:", x)
    for y in range(len(conditions[x])):
        print("y:", y)

        # Create msg, draw and flip @TODO function this up for each screen
        msg0_msg = visual.TextStim(win0, text=conditions[x][y][0])
        msg1_msg = visual.TextStim(win1, text=conditions[x][y][1])
        msg0_msg.draw()
        msg1_msg.draw()
        win0.flip()
        win1.flip()     

        # set a clock and wait for a second
        clock = core.Clock()
        while clock.getTime() < 1.0:  # Clock times are in seconds
            pass
            
# Finish and close the windows
print("Fin")
win0.close()
win0.close()