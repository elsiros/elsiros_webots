# This module has to be launched from referee.py by command:
#external_controllers = subprocess.Popen(['python', 'start_teams.py'])
# before termination referee.py has to terminate subprocess:
#external_controllers.terminate()


import datetime
import os
import subprocess
from pathlib import Path
import json

# following 3 lines provide minimizing of console in Windows
# you can comment them if you need console window.
import win32gui, win32con
t = win32gui.GetForegroundWindow()
win32gui.ShowWindow(t, win32con.SW_MINIMIZE)

with open('game.json', "r") as f:
    game_data = json.loads(f.read())

red_team_config_file = game_data['red']['config']
blue_team_config_file = game_data['blue']['config']

with open(red_team_config_file, "r") as f:
    red_team_data = json.loads(f.read())

with open(blue_team_config_file, "r") as f:
    blue_team_data = json.loads(f.read())

current_working_directory = Path.cwd()
red_team_controller_subdirectory = Path(red_team_data['robotStartCmd']).parents[0]
red_team_controller_filename = Path(red_team_data['robotStartCmd']).name
blue_team_controller_subdirectory = Path(blue_team_data['robotStartCmd']).parents[0]
blue_team_controller_filename = Path(blue_team_data['robotStartCmd']).name

os.chdir(current_working_directory.parent/red_team_controller_subdirectory)

role = red_team_data['players']['1']['role']

port01 = str(game_data['red']['ports'][0])
filename01 = "output" + f"{port01}"+ ".txt" 
with open(filename01, "w") as f01:
    print(datetime.datetime.now(), file = f01)
    if Path(red_team_data['robotStartCmd']).suffix == '.py':
        p01 = subprocess.Popen(['python', red_team_controller_filename, port01, str(game_data['red']['id']),
                              'red', '1', red_team_data['players']['1']['role']], stderr=f01)
    else:
        p01 = subprocess.Popen([red_team_controller_filename, port01, str(game_data['red']['id']), 'red', '1',
                               red_team_data['players']['1']['role']], stderr=f01)

port02 = str(game_data['red']['ports'][1])
filename02 = "output" + f"{port02}"+ ".txt"
with open(filename02, "w") as f02:
    print(datetime.datetime.now(), file = f02)
    if Path(red_team_data['robotStartCmd']).suffix == '.py':
        p02 = subprocess.Popen(['python', red_team_controller_filename, port02, str(game_data['red']['id']),
                                  'red', '2', red_team_data['players']['2']['role']], stderr=f02)
    else:
        p02 = subprocess.Popen([red_team_controller_filename, port02, str(game_data['red']['id']),
                                  'red', '2', red_team_data['players']['2']['role']], stderr=f02)

os.chdir(current_working_directory.parent/blue_team_controller_subdirectory)

port21 = str(game_data['blue']['ports'][0])
filename21 = "output" + f"{port21}"+ ".txt"
with open(filename21, "w") as f21:
    print(datetime.datetime.now(), file = f21)
    if Path(blue_team_data['robotStartCmd']).suffix == '.py':
        p21 = subprocess.Popen(['python', blue_team_controller_filename, port21 , str(game_data['blue']['id']),
                                  'blue', '1', blue_team_data['players']['1']['role']],  stderr=f21)
    else:
        p21 = subprocess.Popen([blue_team_controller_filename, port21 , str(game_data['blue']['id']),
                            'blue', '1', blue_team_data['players']['1']['role']],  stderr=f21)

port22 = str(game_data['blue']['ports'][1])
filename22 = "output" + f"{port22}"+ ".txt"
with open(filename22, "w") as f22:
    print(datetime.datetime.now(), file = f22)
    if Path(blue_team_data['robotStartCmd']).suffix == '.py':
        p22 = subprocess.Popen(['python', blue_team_controller_filename, port22, str(game_data['blue']['id']),
                                  'blue', '2', blue_team_data['players']['2']['role']], stderr=f22)
    else:
        p22 = subprocess.Popen([blue_team_controller_filename, port22, str(game_data['blue']['id']),
                                  'blue', '2', blue_team_data['players']['2']['role']], stderr=f22)

p01.wait()
p02.wait()
p21.wait()
p22.wait()