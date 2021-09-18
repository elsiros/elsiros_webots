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

with open('output10001.txt', "w") as f1001:
    print(datetime.datetime.now(), file = f1001)
    p10001 = subprocess.Popen(['python', red_team_controller_filename, str(game_data['red']['ports'][0]), str(game_data['red']['id']),
                              'red', '1', red_team_data['players']['1']['role']], stderr=f1001)

with open('output10002.txt', "w") as f1002:
    print(datetime.datetime.now(), file = f1002)
    p10002 = subprocess.Popen(['python', red_team_controller_filename, str(game_data['red']['ports'][1]), str(game_data['red']['id']),
                              'red', '2', red_team_data['players']['2']['role']], stderr=f1002)

os.chdir(current_working_directory.parent/blue_team_controller_subdirectory)

with open('output10021.txt', "w") as f1021:
    print(datetime.datetime.now(), file = f1021)
    p10021 = subprocess.Popen(['python', blue_team_controller_filename, str(game_data['blue']['ports'][0]) , str(game_data['blue']['id']),
                              'blue', '1', blue_team_data['players']['1']['role']],  stderr=f1021)

with open('output10022.txt', "w") as f1022:
    print(datetime.datetime.now(), file = f1022)
    p10022 = subprocess.Popen(['python', blue_team_controller_filename, str(game_data['blue']['ports'][1]), str(game_data['blue']['id']),
                              'blue', '2', blue_team_data['players']['2']['role']], stderr=f1022)

p10001.wait()
p10002.wait()
p10021.wait()
p10022.wait()