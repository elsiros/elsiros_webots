import os
import subprocess

path = os.path.dirname(os.path.realpath(__file__))  
path_external = os.path.split(path)[0] + r'/VIA_PB'
subprocess.Popen(path_external+'/Package.bat', cwd=path_external)

