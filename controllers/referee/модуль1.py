import os
import signal
import subprocess

external_controllers_process = subprocess.Popen(['python', 'start_teams.py'])

input('press Enter')
subprocess.Popen("TASKKILL /F /PID {pid} /T".format(pid=external_controllers_process.pid))
#external_controllers_process.kill()
