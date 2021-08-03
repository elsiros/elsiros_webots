import sys
import os
import math
import json
import time


current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if sys.version == '3.4.0':
    # will be running on openMV
    SIMULATION = 2                   #  2 - live on openMV
else:
    # will be running on desktop computer
    current_work_directory += '/'
    import threading
    SIMULATION = 4                   # 0 - Simulation without physics, 
                                     # 1 - Simulation synchronous with physics, 
                                     # 3 - Simulation streaming with physics
                                     # 4 - Simulation in Webots

try:
    sys.path.append( current_work_directory + 'Soccer/')
    sys.path.append( current_work_directory + 'Soccer/Motion/')
    sys.path.append( current_work_directory + 'Soccer/Localisation/')
    sys.path.append( current_work_directory)
        
    from class_Glob import Glob
    from class_Local import *
    from strategy import Player

    if SIMULATION == 4:
        from class_Motion_Webots_inner import Motion_sim as Motion

        initial_coord = [-0.5, 0, 0]
        glob = Glob(SIMULATION, current_work_directory)
        glob.pf_coord = initial_coord
        motion = Motion(glob)
        motion.sim_Start()
        motion.direction_To_Attack = -initial_coord[2]
        motion.activation()
        local = Local(motion, glob, coord_odometry = initial_coord)
        motion.local = local
        local.coordinate_record(odometry = True)
        motion.falling_Flag = 0
        second_pressed_button = 1
        player = Player('forward', second_pressed_button, glob, motion, local)
        player.play_game()

    elif SIMULATION == 0 or SIMULATION == 1 or SIMULATION == 3:
        from class_Motion import *
        from class_Motion_sim import*
        from class_Motion_sim import Motion_sim as Motion

        clientID = []
        transfer_Datas =[]
        motion =[]
        local =[]
        glob = []
        #vision = []
        robots_Number = 2
        robot_IDs = [ '', '#0', '#1', '#2']
        initial_coord = [[0.2, 0, 0], [ -1.8, 0, 0], [-1.1, 0, 0], [-1.8, 0, 0]]
        clientID.append(sim_Enable('127.0.0.1', -19997))
        print('clientID1 =', clientID[0])
        if robots_Number > 1:
            clientID.append(sim_Enable('127.0.0.2', -19998))
            print('clientID2 =', clientID[1])
        if robots_Number > 2:
            clientID.append(sim_Enable('127.0.0.3', -19999))
            print('clientID3 =', clientID[2])
        if robots_Number > 3:
            clientID.append(sim_Enable('127.0.0.4', -20000))
            print('clientID4 =', clientID[3])

        events = []
        t = []
        m =[]
        lock = threading.Lock()
        transfer_Data = Transfer_Data()
        for i in range(robots_Number):
            glob.append(Glob(SIMULATION, current_work_directory))
            glob[i].pf_coord = initial_coord[i]
            events.append(threading.Event())
            transfer_Datas.append(transfer_Data)
            #vision.append(Vision(glob[i]))
            motion.append(Motion(glob[i], clientID[i] , events[i], lock, transfer_Datas[i], robots_Number, robot_Number = robot_IDs[i]))
            motion[i].sim_Start()
            motion[i].direction_To_Attack = -initial_coord[i][2]
            motion[i].activation()
            local.append(Local(motion[i], glob[i], coord_odometry = initial_coord[i]))
            motion[i].local = local[i]
            local[i].coordinate_record(odometry = True)
            motion[i].falling_Flag = 0
            #goalkeeper, forward_v2, run_test, penalty_Shooter, rotation_test, penalty_Goalkeeper, spot_walk, dance, quaternion_test
            if i == 0 or i == 3:
                second_pressed_button = 1
                m.append(Player('penalty_Shooter', second_pressed_button, glob[i], motion[i], local[i])) 
            if i == 1:
                second_pressed_button = 1
                m.append(Player('penalty_Goalkeeper', second_pressed_button, glob[i], motion[i], local[i]))
            if i == 2:
                second_pressed_button = 1
                m.append(Player('forward_v2', second_pressed_button, glob[i], motion[i], local[i]))
            
        if  SIMULATION == 1 :
            t0 = threading.Thread( target = simulation_Trigger_Accumulator, args=(clientID, events, transfer_Datas, lock))
        for i in range(robots_Number):
            t.append(threading.Thread( target = m[i].play_game, args=()))
        if  SIMULATION == 1 :
            t0.setDaemon(True)
            t0.start()
        for i in range(robots_Number): t[i].start()


except Exception as e:
    if SIMULATION == 4:
        f = open("Error.txt",'w')
        sys.print_exception(e,f)
        f.close()
        sys.print_exception(e,sys.stdout)
    else:
        print(e)
