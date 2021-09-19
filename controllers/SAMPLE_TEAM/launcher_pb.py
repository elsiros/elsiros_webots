"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of Azer Babaev.
The module is designed for strategy of soccer game by forward and goalkeeper.
"""

import time
import logging
from gcreceiver import ThreadedGameStateReceiver
from Soccer.Localisation.class_Glob import Glob
from Soccer.Localisation.class_Local import *
from Soccer.strategy import Player
from Soccer.Motion.class_Motion_Webots_PB import Motion_sim


def init_gcreceiver(team, player, is_goalkeeper):
    """
    The function creates and object receiver of Game Controller messages. Game Controller messages are broadcasted to 
    teams and to referee. Format of messages can be seen in module gamestate.py. Messages from Game Controller 
    contains Robot info, Team info and Game state info.
    usage of function:
        object: receiver = init_gcreceiver(int: team, int: player, bool: is_goalkeeper)
            team - number of team id. For junior competitions it is recommended to use unique id
                   for team in range 60 - 127
            player - number of player displayed at his trunk
            is_goalkeeper - True if player is appointed to play role of goalkeeper
    """
    receiver = ThreadedGameStateReceiver(team, player, is_goalkeeper)    
    receiver.start() # Strat receiving and answering
    return receiver

def player_super_cycle(falling, team_id, robot_color, player_number, SIMULATION, current_work_directory, robot, pause, logger):
    """
    The function is called player_super_cycle because during game player can change several roles. Each role
    appointed to player put it into cycle connected to playing it's role. Cycles of roles are defined in strategy.py
    module. player_super_cycle is cycle of cycles. For example player playing role of 'forward' can change role to
    'penalty_shooter' after main times and extra times of game finished. In some situations you may decide to switch
    roles between forward player and goalkeeper. 
    Usage:
        player_super_cycle(object: falling, int: team_id, str: robot_color, int: player_number, int: SIMULATION,
                            Path_object: current_work_directory, object: robot, object: pause)
        falling - class object which contains int: falling.Flag which is used to deliver information about falling from
                    low level logic to high level logic. falling.Flag can take 
                    0 - nothing happend, 1 -falling on stomach, -1 - falling face up,
                    2 - falling to left, -2 - falling to right, 3 - exit from playing fase
        team_id - can take value from 60 to 127
        robot_color - can be 'red' or 'blue'
        player_number - can be from 1 to 5, with 1 to be assigned to goalkeeper
        SIMULATION    - used for definition of simulation enviroment. value 4 is used for Webots simulation,
                        value 2 is used for playing in real robot
        current_work_directory - is Path type object 
        robot       - object of class which is used for communication between robot model in simulation and controller
                      program. In case of external controller program 'ProtoBuf' communication manager is used. 
                      'ProtoBuf' - is protocol developed by Google.
        pause       - object of class Pause which contains pause.Flag boolean variable. It is used to transfer pressing 
                      pause button on player's dashboard event to player's high level logic. 


    """
    with open('../referee/game.json', "r") as f:        # extracting data from game.json file which have 
        game_data = json.loads(f.read())                # to be created by human referee

    with open('../referee/' + game_data[ robot_color]['config'], "r") as f:     # extracting data from team.json file 
        team_data = json.loads(f.read())
    
    # below  4 lines are initial coordinates of players depending on game fase
    initial_coord_goalkeeper  = team_data['players']['1']['readyStartingPose']['pf_coord'] 
    initial_coord_forward = team_data['players']['2']['readyStartingPose']['pf_coord']
    initial_coord_goalkeeper_at_penalty = team_data['players']['1']['goalKeeperStartingPose']['pf_coord']
    initial_coord_forward_at_penalty = team_data['players']['2']['shootoutStartingPose']['pf_coord']
    if player_number == 1: is_goalkeeper = True
    else: is_goalkeeper = False
    receiver = init_gcreceiver(team_id, player_number, is_goalkeeper)
    robot.receiver = receiver
    former_game_state = 'STATE_SET'
    former_player_penalty = 0
    logger.info('waiting for game controller launch')
    playing_allowed = False
    current_secondary_state = None
    while True:                                 # this is main cycle of supercycle
        #if receiver.team_state == None:
        #    if current_secondary_state == 'STATE_PENALTYSHOOT':
        #        print('simulator reset')
        #        robot.simulationReset()
        seconds = 0
        while True:
            if receiver.team_state == None:
                if seconds == 0:
                    message = '\n Game Controller is not launched. Waiting..'
                    logger.info(message)
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
                        message = '\n Game Controller STATE_INITIAL. Waiting for READY..'
                        logger.info(message)
                    else:
                        print('.', end = '')
                    seconds += 1
                else:
                    break
            #robot.step(200)
            time.sleep(0.2)
        seconds = 0
        while True:
            if receiver.team_state != None:
                if receiver.state.game_state == 'STATE_READY':
                    former_game_state = 'STATE_READY'
                    if seconds == 0:
                        message = '\n Game Controller STATE_READY. Waiting for SET..'
                        logger.info(message)
                    else:
                        print('.', end = '')
                    seconds += 1
                else:
                    break
            #robot.step(200)
            time.sleep(0.2)
        seconds = 0
        while True:
            if receiver.team_state != None:
                if receiver.state.game_state == 'STATE_SET':
                    former_game_state = 'STATE_SET'
                    if seconds == 0:
                        message = '\n Game Controller STATE_SET. Waiting for PLAYING..'
                        logger.info(message)
                    else:
                        print('.', end = '')
                    seconds += 1
                else:
                    break
            #robot.step(200)
            time.sleep(0.2)
        if receiver.team_state != None:
            current_game_state = receiver.state.game_state
            current_secondary_state = receiver.state.secondary_state
            current_player_penalty = receiver.player_state.penalty
            if current_game_state == 'STATE_PLAYING' and current_secondary_state != 'STATE_PENALTYSHOOT' and current_player_penalty== 0:
                second_pressed_button = 1
                logger.info('start playing')
                if former_game_state == 'STATE_SET':
                    if receiver.state.kick_of_team != receiver.team_state.team_number:
                        second_pressed_button = 4
                    logger.info('former_game_state == "STATE_SET"')
                    if player_number == 1:
                        initial_coord = initial_coord_goalkeeper
                        if receiver.team_state.team_color == 'BLUE':
                            role = 'goalkeeper_old_style'
                        else:
                            role = 'goalkeeper'
                        playing_allowed = True
                        logger.info('playing allowed')
                    else: 
                        initial_coord = initial_coord_forward
                        if receiver.team_state.team_color == 'BLUE':
                            role = 'forward_old_style'
                        else:
                            role = 'forward'
                        playing_allowed = True
                        logger.info('playing allowed')
                if former_player_penalty !=0:
                    statement1 = 2* (player_number == 1) - 1
                    statement2 = 2* (receiver.state.first_half) - 1
                    statement3 = 2* (receiver.team_state.team_color == 'RED') -1
                    statement4 = 2* (game_data['side_left'] == game_data['blue']['id']) - 1
                    if statement1 * statement2 * statement3 * statement4 == 1:
                        initial_coord = [-0.9, 1.3, -math.pi/2]
                    else:
                        initial_coord = [-0.9, -1.3, math.pi/2]
                    playing_allowed = True
                    second_pressed_button = 1
                    logger.info('playing allowed')
            elif current_game_state == 'STATE_PLAYING' and current_secondary_state == 'STATE_PENALTYSHOOT' and current_player_penalty== 0:
                second_pressed_button = 1
                #print('start playing')
                if former_game_state == 'STATE_SET':
                    logger.info('former_game_state == "STATE_SET"')
                    if player_number == 1:
                        initial_coord = initial_coord_goalkeeper_at_penalty
                        #if receiver.team_state.team_color == 'BLUE':
                        #    role = 'goalkeeper_old_style'
                        #else:
                        role = 'penalty_Goalkeeper'
                        playing_allowed = True
                        logger.info('playing allowed')
                    else: 
                        initial_coord = initial_coord_forward_at_penalty
                        #if receiver.team_state.team_color == 'BLUE':
                        #    role = 'forward_old_style'
                        #else:
                        role = 'penalty_Shooter'
                        playing_allowed = True
                        logger.info('playing allowed')
            if playing_allowed:
                logger.info ('current_game_state =' + str(current_game_state) + ' current_player_penalty =' + str(current_player_penalty))
                logger.info ('former_game_state ='+ str(former_game_state) + ' former_player_penalty =' + str(former_player_penalty))
                glob = Glob(SIMULATION, current_work_directory)
                glob.pf_coord = initial_coord
                motion = Motion_sim(glob, robot, receiver, pause, logger)
                motion.sim_Start()
                motion.direction_To_Attack = -initial_coord[2]
                motion.activation()
                local = Local(logger, motion, glob, coord_odometry = initial_coord)
                motion.local = local
                local.coordinate_record()
                motion.falling_Flag = 0
                player = Player(logger, role, second_pressed_button, glob, motion, local)
                player.play_game()
                playing_allowed = False
            former_game_state = receiver.state.game_state
            former_player_penalty = receiver.player_state.penalty
            time.sleep(0.02)






