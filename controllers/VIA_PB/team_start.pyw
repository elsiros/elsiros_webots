import datetime
import os
import subprocess

with open('output10001.txt', "a") as f1001:
    print(datetime.datetime.now(), file = f1001)
    subprocess.Popen(['python', 'main_pb.py', '10001'], stderr=f1001)
with open('output10002.txt', "a") as f1002:
    print(datetime.datetime.now(), file = f1002)
    subprocess.Popen(['python', 'main_pb.py', '10002'], stderr=f1002)

