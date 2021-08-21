"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of Azer Babaev.
The module is designed for strategy of soccer game by forward and goalkeeper.
"""

import time
from gcreceiver import ThreadedGameStateReceiver
from Soccer.Localisation.class_Glob import Glob
from Soccer.Localisation.class_Local import *
from Soccer.strategy import Player
from Soccer.Motion.class_Motion_Webots_inner import Motion_sim


def init_gcreceiver(team, player, is_goalkeeper):
    receiver = ThreadedGameStateReceiver(team, player, is_goalkeeper)    
    receiver.start() # Strat receiving and answering
    return receiver

def player_super_cycle(falling, team, player_number, SIMULATION, current_work_directory, robot, pause):
    if player_number == 1: is_goalkeeper = True
    else: is_goalkeeper = False
    receiver = init_gcreceiver(team, player_number, is_goalkeeper)
    former_game_state = 'STATE_SET'
    former_player_penalty = 0
    print('waiting for game controller launch')
    playing_allowed = False
    while True:
        seconds = 0
        while True:
            if receiver.team_state == None:
                if seconds == 0:
                    message = '\nGame Controller is not launched. Waiting..'
                    print(message, end = '')
                else:
                    print('.', end = '')
                seconds += 1
            else: 
                break
            time.sleep(1)
        seconds = 0
        while True:
            if receiver.team_state != None:
                if receiver.state.game_state == 'STATE_INITIAL':
                    former_game_state = 'STATE_INITIAL'
                    if seconds == 0:
                        message = '\nGame Controller STATE_INITIAL. Waiting for READY..'
                        print(message, end = '')
                    else:
                        print('.', end = '')
                    seconds += 1
                else:
                    break
            robot.step(200)
        seconds = 0
        while True:
            if receiver.team_state != None:
                if receiver.state.game_state == 'STATE_READY':
                    former_game_state = 'STATE_READY'
                    if seconds == 0:
                        message = '\nGame Controller STATE_READY. Waiting for SET..'
                        print(message, end = '')
                    else:
                        print('.', end = '')
                    seconds += 1
                else:
                    break
            robot.step(200)
        seconds = 0
        while True:
            if receiver.team_state != None:
                if receiver.state.game_state == 'STATE_SET':
                    former_game_state = 'STATE_SET'
                    if seconds == 0:
                        message = '\nGame Controller STATE_SET. Waiting for PLAYING..'
                        print(message, end = '')
                    else:
                        print('.', end = '')
                    seconds += 1
                else:
                    break
            robot.step(200)
        if receiver.team_state != None:
            current_game_state = receiver.state.game_state
            current_secondary_state = receiver.state.secondary_state
            current_player_penalty = receiver.player_state.penalty
            if current_game_state == 'STATE_PLAYING' and current_secondary_state != 'STATE_PENALTYSHOOT' and current_player_penalty== 0:
                second_pressed_button = 1
                print('start playing')
                if former_game_state == 'STATE_SET':
                    if receiver.state.kick_of_team != receiver.team_state.team_number:
                        second_pressed_button = 4
                    print('former_game_state == "STATE_SET"')
                    if player_number == 1:
                        initial_coord = [-1.68, 0, 0]
                        if receiver.team_state.team_color == 'BLUE':
                            role = 'goalkeeper_old_style'
                        else:
                            role = 'goalkeeper'
                        playing_allowed = True
                        print('playing allowed')
                    else: 
                        initial_coord = [-0.4, 0, 0]
                        if receiver.team_state.team_color == 'BLUE':
                            role = 'forward_old_style'
                        else:
                            role = 'forward'
                        playing_allowed = True
                        print('playing allowed')
                if former_player_penalty !=0:
                    statement1 = 2* (player_number == 1) - 1
                    statement2 = 2* (receiver.state.first_half) - 1
                    statement3 = 2* (receiver.team_state.team_color == 'RED') -1
                    if statement1 * statement2 * statement3 == 1:
                        initial_coord = [-0.9, 1.3, -math.pi/2]
                    else:
                        initial_coord = [-0.9, -1.3, math.pi/2]
                    playing_allowed = True
                    second_pressed_button = 1
                    print('playing allowed')
            elif current_game_state == 'STATE_PLAYING' and current_secondary_state == 'STATE_PENALTYSHOOT' and current_player_penalty== 0:
                second_pressed_button = 1
                #print('start playing')
                if former_game_state == 'STATE_SET':
                    print('former_game_state == "STATE_SET"')
                    if player_number == 1:
                        initial_coord = [-1.68, 0, 0]
                        #if receiver.team_state.team_color == 'BLUE':
                        #    role = 'goalkeeper_old_style'
                        #else:
                        role = 'penalty_Goalkeeper'
                        playing_allowed = True
                        print('playing allowed')
                    else: 
                        initial_coord = [0.2, 0, 0]
                        #if receiver.team_state.team_color == 'BLUE':
                        #    role = 'forward_old_style'
                        #else:
                        role = 'penalty_Shooter'
                        playing_allowed = True
                        print('playing allowed')
            if playing_allowed:
                print ('current_game_state =', current_game_state, 'current_player_penalty =', current_player_penalty)
                print ('former_game_state =', former_game_state, 'former_player_penalty =', former_player_penalty)
                glob = Glob(SIMULATION, current_work_directory)
                glob.pf_coord = initial_coord
                motion = Motion_sim(glob, robot, receiver, pause)
                motion.sim_Start()
                motion.direction_To_Attack = -initial_coord[2]
                motion.activation()
                local = Local(motion, glob, coord_odometry = initial_coord)
                motion.local = local
                local.coordinate_record(odometry = True)
                motion.falling_Flag = 0
                player = Player(role, second_pressed_button, glob, motion, local)
                player.play_game()
                playing_allowed = False
            former_game_state = receiver.state.game_state
            former_player_penalty = receiver.player_state.penalty
            robot.step(20)
        





