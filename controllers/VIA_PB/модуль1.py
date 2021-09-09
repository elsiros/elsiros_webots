
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
sys.path.append(current_dir)
sys.path.append(str(current_dir))
print(current_dir) 
print(str(current_dir)) 
print(sys.path)
