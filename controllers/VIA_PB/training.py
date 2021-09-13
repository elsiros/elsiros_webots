# This module is used for tuning of robot
# must be used with training world



import datetime
import os
import subprocess
from pathlib import Path
import json

# following 3 lines provide minimizing of console in Windows
#import win32gui, win32con
#t = win32gui.GetForegroundWindow()
#win32gui.ShowWindow(t, win32con.SW_MINIMIZE)


role = 'run_test'
second_pressed_button = '9'
initial_coord = '[-1.8, 0, 0]'

with open('output10001.txt', "a") as f1001:
    print(datetime.datetime.now(), file = f1001)
    p10001 = subprocess.Popen(['python', 'main_pb.py', '10001', '-1', 'red', '1', role, second_pressed_button, initial_coord], stderr=f1001)

