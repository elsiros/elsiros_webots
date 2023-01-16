"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of Azer Babaev.
The module is designed for creating players' dashboard and alternating between 
team game with Game Controller or individual play without Game Controller.
"""
"""
The module must be used for running with arguments of command line:
Order of arguments: port, team_id, robot_color, robot_number, role, second_pressed_button, initial_coord
all arguments have to be transferred as str values with following type assignmet to other types:
int: port  - TCP port of communication with player.exe, must be defined in team.json
int: team_id  - value between 60 -127, must be appointed by human referee
str: robot_color - can be 'red' or 'blue', defined in game.json
int: robot_number - from 1 to 5. Must be defined in team.json
str: role  - initial role of player in game. Possible values of role argument:
             'forward', 'goalkeeper', 'penalty_Shooter', 'penalty_Goalkeeper', 'run_test'
int: second_pressed_button  - possible values of 2-nd argument: 1-8. 
                            At real robot second pressed button number is used to choose mode of role 
list: initial_coord - initial localization and orientation values of robot transferred to controller
                      by human handler of real robot or by auto-referee to controllor in simulation. 
                      Elenets of list: [float: x, float: y, float: yaw]. x and y - coordinates expressed in m.
                      yaw - orientation of robot in radians.
"""

import sys
import os
import math
import json
import time
import wx
import threading
import logging
from pathlib import Path

# Uncomment one of 4 following lines to set proper logging level

#LOGGING_LEVEL = logging.ERROR              # logging will be with fewest number of messages
#LOGGING_LEVEL = logging.WARNING            # logging with low number of messages
LOGGING_LEVEL = logging.INFO               # logging with moderate messaging level
#LOGGING_LEVEL = logging.DEBUG              # logging with large number of messages. Simulation will be slow.


SIMULATION = 4                       # 0 - Simulation without physics in Coppelia, 
                                     # 1 - Simulation synchronous with physics in Coppelia, 
                                     # 2 - used for real robot
                                     # 3 - Simulation streaming with physics in Coppelia
                                     # 4 - Simulation in Webots

current_work_directory = Path.cwd()
from Soccer.Localisation.class_Glob import Glob
from Soccer.Localisation.class_Local import *
from Soccer.strategy import Player
from Soccer.Motion.class_Motion_Webots_PB import Motion_sim
from launcher_pb import *
sys.path.append(str(current_work_directory.parent/'player'))
import google
import google.protobuf
from google.protobuf import descriptor
from communication_manager_robokit import CommunicationManager

with open('../referee/game.json', "r") as f:
    game_data = json.loads(f.read())

with open('../referee/' + game_data['red']['config'], "r") as f:
    team_1_data = json.loads(f.read())

with open('../referee/' + game_data['blue']['config'], "r") as f:
    team_2_data = json.loads(f.read())


class Log:
    def __init__(self, filename):
        self.filename = filename
        self.log_format_file = f"%(asctime)s - [%(levelname)s] - %(name)s - (%(filename)s).%(funcName)s(%(lineno)d) - %(message)s"
        self.log_format_console = f"%(message)s"
        self.file_level = logging.DEBUG
        self.stream_level = logging.INFO

    def get_file_handler(self):
        file_handler = logging.FileHandler(self.filename)
        file_handler.setLevel(self.file_level)
        file_handler.setFormatter(logging.Formatter(self.log_format_file))
        return file_handler

    def get_stream_handler(self):
        stream_handler = logging.StreamHandler(sys.stdout)
        stream_handler.setLevel(self.stream_level)
        stream_handler.setFormatter(logging.Formatter(self.log_format_console))
        return stream_handler

    def get_logger(self, name):
        logger = logging.getLogger(name)
        logger.setLevel(LOGGING_LEVEL)
        logger.addHandler(self.get_file_handler())
        logger.addHandler(self.get_stream_handler())
        return logger



class Falling:
    def __init__(self):
        self.Flag = 0

class Pause:
    def __init__(self):
        self.Flag = False
global pause
pause = Pause()

def main_procedure():
    global pause
    global log
    global logger
    Port = sys.argv[1]
    logger.info('port = %s', Port)
    logarg =  log.get_logger('communication_manager')
    robot = CommunicationManager(1, '127.0.0.1', int(Port), logarg, team_color=sys.argv[3].upper(), player_number = int(sys.argv[4]), time_step = 25)

    falling = Falling()

    team_id = int(sys.argv[2])
    role = sys.argv[5]
    logger.info('role = %s', role)
    if team_id > 0:
        robot_color = sys.argv[3]
        robot_number = int(sys.argv[4])
        player_super_cycle(falling, team_id, robot_color, robot_number, SIMULATION, current_work_directory, robot, pause, logger)

    second_pressed_button = int(sys.argv[6])
    initial_coord = eval(sys.argv[7])
    logger.debug(initial_coord)
    logger.info('Player is going to play without Game Controller')
    glob = Glob(SIMULATION, current_work_directory)
    glob.pf_coord = initial_coord
    motion = Motion_sim(glob, robot, None, pause, logger)
    motion.sim_Start()
    motion.direction_To_Attack = -initial_coord[2]
    time.sleep(1)
    motion.activation()
    local = Local(logger, motion, glob, coord_odometry = initial_coord)
    motion.local = local
    local.coordinate_record()
    motion.falling_Flag = 0
    player = Player(logger, role, second_pressed_button, glob, motion, local)
    timer1 = robot.current_time
    logger.debug( 'start time: %i',timer1)
    player.play_game()
    logger.debug( 'total time: %i', robot.current_time - timer1)
    sys.exit(0)


class RedirectText(object):
    def __init__(self,aWxTextCtrl):
        self.out = aWxTextCtrl

    def write(self,string):
        self.out.WriteText(string)

class Main_Panel(wx.Frame):
    def __init__(self, *args, **kwargs):
        super(Main_Panel, self).__init__(*args, **kwargs)
        #self.main_procedure()
        self.InitUI()
        wx.CallLater(1000, self.main_procedure)

    def main_procedure(self):
        t1 = threading.Thread( target = main_procedure, args=())
        t1.setDaemon(True)
        t1.start()

    def InitUI(self):
        self.console_Panel = wx.Panel(self)
        panel = wx.Panel(self.console_Panel)
        self.log = wx.TextCtrl(self.console_Panel, -1, style=wx.TE_MULTILINE) #|wx.TE_READONLY) #|wx.HSCROLL)
        log_box = wx.BoxSizer(wx.VERTICAL)
        log_box.Add(panel,0,wx.TOP)
        log_box.Add(self.log, proportion = 1, flag=wx.EXPAND|wx.BOTTOM|wx.TOP)
        self.console_Panel.SetSizer(log_box)
        redir = RedirectText(self.log)
        sys.stdout = redir

        hbox = wx.BoxSizer()
        sizer = wx.GridSizer(1, 2, 2, 2)

        btn1 = wx.Button(panel, label='Exit')
        btn2 = wx.Button(panel, label='Pause')

        sizer.AddMany([btn1, btn2])

        hbox.Add(sizer, 0, wx.TOP)
        panel.SetSizer(hbox)


        btn1.Bind(wx.EVT_BUTTON, self.ShowMessage1)
        btn2.Bind(wx.EVT_BUTTON, self.ShowMessage2)

        self.SetSize((300, 200))
        robot_color = sys.argv[3]
        robot_number = sys.argv[4]
        title = 'Team ' + robot_color + ' player '+ robot_number
        self.SetTitle(title)
        width, height = wx.GetDisplaySize().Get()
        if robot_color == 'red':
            x_position = width - 300 * (5- int(robot_number))
        else:
            x_position = width - 300 * (3- int(robot_number))
        self.SetPosition((x_position, height -225))
        #self.Centre()

    def ShowMessage1(self, event):
        global logger
        logger.info('Exit button pressed')
        sys.stdout = sys.__stdout__
        sys.exit(0)

    def ShowMessage2(self, event):
        global logger
        global pause
        if pause.Flag:
            pause.Flag = False
        else:
            pause.Flag = True
        logger.info('Pause button pressed')


def main():
    app = wx.App()
    ex = Main_Panel(None)
    ex.Show()
    filename = "output" + str(sys.argv[1]) + ".txt"
    global log
    log = Log(filename)
    global logger
    logger = log.get_logger(__name__)
    app.MainLoop()

main()



