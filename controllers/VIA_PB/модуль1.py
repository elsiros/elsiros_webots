
import sys
import os
import math
import json
import time
import wx
import threading
from pathlib import Path

current_work_directory = os.getcwd().replace('\\', '/') + '/'
#print(current_work_directory)

#current_dir = Path.cwd()
 
#print(current_dir.parent/'referee'/'game.json')
#with open(current_dir.parent/'referee'/'game.json', "r") as f:
#    game_data = json.loads(f.read())
#print(game_data)

dir1 = 'C:/Users/ab/Documents/GitHub/elsiros_webots/protos/Robokit1/Robokit1_meshes/'
p = Path(dir1)
print(p)
files = sorted(Path(p).glob('*.*'))
#print(files)
for file in files:
    new_file_name = p.with_name(file.name.replace('Kondo006', 'Robokit1'))
    new_file_name = p.with_name(new_file_name.name.replace('Kondo007', 'Robokit1'))
    print(new_file_name)
    file.rename(new_file_name)
#for file in files:
#    new_file_name = p.with_name(file.name.replace('Kondo007', 'Robokit1'))
#    print(new_file_name)