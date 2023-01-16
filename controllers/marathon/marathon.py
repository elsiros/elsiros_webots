
import datetime
import os
import subprocess
from pathlib import Path
import json
from controller import Supervisor, AnsiCodes, Node
import time
import math

supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

robot_translation = supervisor.getFromDef('BLUE_PLAYER_1').getField('translation')

current_working_directory = Path.cwd()

def uprint(*text):
    with open(str(current_working_directory) + "\Marathon_log.txt",'a') as f:
        print(*text, file = f)
    print(*text )

os.chdir(current_working_directory.parent/'Robofest_TEAM')

role01 = 'marathon' 
second_pressed_button = '4'
initial_coord = '[0.0, 0, 0]'
robot_color = 'blue'
robot_number = '1'
team_id = '-1'          # value -1 means game will be playing without Game Controller
port01 = '7001'
filename01 = "output" + f"{port01}"+ ".txt"
with open(filename01, "w") as f01:
    print(datetime.datetime.now(), file = f01)
    p01 = subprocess.Popen(['python', 'main_pb.py', port01, team_id, robot_color, robot_number, role01, second_pressed_button, initial_coord], stderr=f01)

distance_count = 0
checkpoints = [0.3, 0, -0.3, 0]
checkpoint_pass = [False, False, False, False]

while supervisor.step(time_step) != -1 :
    #message = 'robot position: ' + str(robot_translation.getSFVec3f()) + 'step: ' + str(supervisor.step(time_step))
    #print(message)
    distance_count += 1
    x_coordinate = robot_translation.getSFVec3f()[0]
    y_coordinate = robot_translation.getSFVec3f()[1]
    path_radius = math.sqrt(x_coordinate**2 + y_coordinate**2)
    if path_radius > 1.3 or path_radius < 0.3 :
        uprint(datetime.datetime.now(), 'distance was NOT finished due to failure ')
        break
    if math.prod(checkpoint_pass):
        uprint(datetime.datetime.now(), 'distance was finished within timesteps: ', distance_count)
        break
    if x_coordinate >= checkpoints[0] and not checkpoint_pass[0] : checkpoint_pass[0] = True
    if x_coordinate <= checkpoints[1] and checkpoint_pass[0] : checkpoint_pass[1] = True
    if x_coordinate <= checkpoints[2] and checkpoint_pass[1] : checkpoint_pass[2] = True
    if x_coordinate >= checkpoints[3] and checkpoint_pass[2] : checkpoint_pass[3] = True

p01.terminate()
supervisor.simulationReset()
supervisor.step(time_step)
supervisor.simulationSetMode(supervisor.SIMULATION_MODE_PAUSE)
#supervisor.worldReload()