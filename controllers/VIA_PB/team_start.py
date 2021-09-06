import datetime
import os
import subprocess
import win32gui, win32con
import time
import math

t = win32gui.GetForegroundWindow()
win32gui.ShowWindow(t, win32con.SW_MINIMIZE)

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

