"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of A. Babaev.

This module is used for tuning of robot
 must be used with training world

 """

import datetime
import os
import subprocess
from pathlib import Path
import json

# following 3 lines provide minimizing of console in Windows
#import win32gui, win32con
#t = win32gui.GetForegroundWindow()
#win32gui.ShowWindow(t, win32con.SW_MINIMIZE)

possible_values_for_role = {
    '1': 'run_test',
    '2': 'forward',
    '3': 'goalkeeper',
    '4': 'forward_old_style',
    '5': 'goalkeeper_old_style',
    '6': 'penalty_Shooter',
    '7': 'penalty_Goalkeeper',
    '8': 'dance'
    }

"""
second_pressed_button can take following values:
if role is 'run_test' then: 
1 - for RUN_TEST_10_STEPS, 
2 - SIDE_STEP_RIGHT_TEST, 
3 - SIDE_STEP_LEFT_TEST, 
4 - for RUN_TEST_20_STEPS
5 - Rotation Right Test, 
6 - Rotation Left Test, 
9 - Spot Walk Test
if role is 'forward' or 'goalkeeper', or 'forward_old_style' or 'goalkeeper_old_style', or 'penalty_Shooter', or 'penalty_Goalkeeper'
then 
1 - start game, 
4 - start game with 10 sec pause.
"""

role01 = possible_values_for_role['1']
second_pressed_button = '4'
initial_coord = '[0.0, 0, 0]'
robot_color = 'red'
robot_number = '1'
team_id = '-1'          # value -1 means game will be playing without Game Controller
port01 = '7001'
filename01 = "output" + f"{port01}"+ ".txt"
with open(filename01, "w") as f01:
    print(datetime.datetime.now(), file = f01)
    p01 = subprocess.Popen(['python', 'main_pb.py', port01, team_id, robot_color, robot_number, role01, second_pressed_button, initial_coord], stderr=f01)

"""
depending of how many players you have at training scene you can uncomment following sections of module
Pay attention to communication port number in robot model controllerArgs 1-st argument something like "7001".
Port number of robot model and in external controller must be equal
"""

#role02 = possible_values_for_role['1']
#second_pressed_button = '1'
#initial_coord = '[0.0, 0, 0]'
#robot_color = 'red'
#robot_number = '2'
#team_id = '-1'          # value -1 means game will be playing without Game Controller
#port02 = '7002'
#filename02 = "output" + f"{port02}"+ ".txt"
#with open(filename02, "w") as f02:
#    print(datetime.datetime.now(), file = f02)
#    p02 = subprocess.Popen(['python', 'main_pb.py', port02, team_id, robot_color, robot_number, role02, second_pressed_button, initial_coord], stderr=f02)


#role21 = possible_values_for_role['1']
#second_pressed_button = '1'
#initial_coord = '[0.0, 0, 0]'
#robot_color = 'blue'
#robot_number = '1'
#team_id = '-1'          # value -1 means game will be playing without Game Controller
#port21 = '7021'
#filename21 = "output" + f"{port21}"+ ".txt"
#with open(filename21, "w") as f21:
#    print(datetime.datetime.now(), file = f21)
#    p21 = subprocess.Popen(['python', 'main_pb.py', port21, team_id, robot_color, robot_number, role21, second_pressed_button, initial_coord], stderr=f21)


#role22 = possible_values_for_role['1']
#second_pressed_button = '1'
#initial_coord = '[0.0, 0, 0]'
#robot_color = 'blue'
#robot_number = '2'
#team_id = '-1'          # value -1 means game will be playing without Game Controller
#port22 = '7022'
#filename22 = "output" + f"{port22}"+ ".txt"
#with open(filename22, "w") as f22:
#    print(datetime.datetime.now(), file = f22)
#    p22 = subprocess.Popen(['python', 'main_pb.py', port22, team_id, robot_color, robot_number, role22, second_pressed_button, initial_coord], stderr=f22)

