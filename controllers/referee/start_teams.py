# This module has to be launched from referee.py by command:
#external_controllers = subprocess.Popen(['python', 'start_teams.py'])
# before termination referee.py has to terminate subprocess:
#external_controllers.terminate()


import datetime
import os
import subprocess
import win32gui, win32con
from pathlib import Path

current_working_directory = Path.cwd()

t = win32gui.GetForegroundWindow()
win32gui.ShowWindow(t, win32con.SW_MINIMIZE)

os.chdir(current_working_directory.parent/'VIA_PB')

with open('output10001.txt', "a") as f1001:
    print(datetime.datetime.now(), file = f1001)
    subprocess.Popen(['python', 'main_pb.py', '10001'], stderr=f1001)

with open('output10002.txt', "a") as f1002:
    print(datetime.datetime.now(), file = f1002)
    subprocess.Popen(['python', 'main_pb.py', '10002'], stderr=f1002)

with open('output10021.txt', "a") as f1021:
    print(datetime.datetime.now(), file = f1021)
    subprocess.Popen(['python', 'main_pb.py', '10021'], stderr=f1021)

with open('output10022.txt', "a") as f1022:
    print(datetime.datetime.now(), file = f1022)
    subprocess.Popen(['python', 'main_pb.py', '10022'], stderr=f1022)

