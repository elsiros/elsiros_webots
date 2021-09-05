
import sys
import os
import math
import json
import time
import wx
import threading
from pathlib import Path

current_work_directory = os.getcwd().replace('\\', '/') + '/'
print(current_work_directory)

current_dir = Path.cwd()
 
print(current_dir.parent/'referee'/'game.json')
with open(current_dir.parent/'referee'/'game.json', "r") as f:
    game_data = json.loads(f.read())
print(game_data)
