import time
from gcreceiver import ThreadedGameStateReceiver
from Soccer.Localisation.class_Glob import Glob
from Soccer.Localisation.class_Local import *
from Soccer.strategy import Player
from Soccer.Motion.class_Motion_Webots_inner import Motion_sim as Motion
from controller import *


def init_gcreceiver(team, player, is_goalkeeper):
    receiver = ThreadedGameStateReceiver(team, player, is_goalkeeper)    
    receiver.start() # Strat receiving and answering
    return receiver

def player_super_cycle(falling, team, player_number, SIMULATION, current_work_directory):
    robot = Supervisor()
    if player_number == 1: is_goalkeeper = True
    else: is_goalkeeper = False
    receiver = init_gcreceiver(team, player_number, is_goalkeeper)
    former_game_state = 'STATE_READY'
    former_player_penalty = 0
    print('waiting for game controller launch')
    while True:
        while True:
            if receiver.team_state != None:
                if receiver.state.game_state == 'STATE_PLAYING' and receiver.player_state.penalty == 0:
                    print('start playing')
                    #if former_game_state == 'STATE_READY':
                    if player_number == 1:
                        initial_coord = [-1.8, 0, 0]
                        role = 'forward'
                    else: 
                        initial_coord = [-0.4, 0, 0]
                        role = 'goalkeeper'
                    second_pressed_button = 1
                    glob = Glob(SIMULATION, current_work_directory)
                    glob.pf_coord = initial_coord
                    motion = Motion(glob, receiver, robot)
                    motion.sim_Start()
                    motion.direction_To_Attack = -initial_coord[2]
                    motion.activation()
                    local = Local(motion, glob, coord_odometry = initial_coord)
                    motion.local = local
                    local.coordinate_record(odometry = True)
                    motion.falling_Flag = 0
                    player = Player(role, second_pressed_button, glob, motion, local)
                    player.play_game()
            if receiver.team_state != None:
                former_game_state = receiver.state.game_state
                former_player_penalty = receiver.player_state.penalty
            time.sleep(1)





