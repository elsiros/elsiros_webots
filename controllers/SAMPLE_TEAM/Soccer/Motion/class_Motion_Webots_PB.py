"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of A. Babaev.
The module is designed to provide communication from motion controller to simulation
"""
import sys, os
import math, time, json

import random

from .class_Motion import *
from .class_Motion_real import Motion_real
from .compute_Alpha_v3 import Alpha

class Motion_sim(Motion_real):
    def __init__(self, glob, robot, gcreceiver, pause):
        self.pause = pause
        self.FRAMELENGTH = 0.02
        import random as random
        self.random = random
        import numpy as np
        self.np = np
        self.Dummy_HData =[]
        self.BallData =[]
        self.timeElapsed = 0
        self.trims = []
        self.jointHandle = []
        self.Dummy_HHandle = None
        self.Dummy_1Handle = None
        self.BallHandle = None
        self.VisionHandle = None
        self.Ballposition = []
        self.sim_step_counter = 0
        self.gcreceiver = gcreceiver
        self.robot = robot
        self.synchronization = False
        self.former_step_time = 0
        self.former_real_time = time.time()
        super().__init__(glob)
        with open(self.glob.current_work_directory / "Init_params" / "Sim_calibr.json", "r") as f:
            data1 = json.loads(f.read())
        self.neck_calibr = data1['neck_calibr']
        self.neck_play_pose = data1['neck_play_pose']
        self.head_pitch_with_horizontal_camera = data1['head_pitch_with_horizontal_camera']
        self.neck_tilt = self.neck_calibr
        self.Vision_Sensor_Display_On = self.glob.params['Vision_Sensor_Display_On']
        self.timestep = 15  
        self.ACTIVEJOINTS = ['Leg_right_10','Leg_right_9','Leg_right_8','Leg_right_7','Leg_right_6','Leg_right_5','hand_right_4',
            'hand_right_3','hand_right_2','hand_right_1','Tors1','Leg_left_10','Leg_left_9','Leg_left_8',
            'Leg_left_7','Leg_left_6','Leg_left_5','hand_left_4','hand_left_3','hand_left_2','hand_left_1','head0','head12']
        self.ACTIVESERVOS = [(10,2),(9,2),(8,2),(7,2),(6,2),(5,2),(4,2),
                (3,2),(2,2),(1,2),(0,2),(10,1),(9,1),(8,1),
                (7,1),(6,1),(5,1),(4,1),(3,1),(2,1),(1,1)]
        self.WBservosList = ["right_ankle_roll", "right_ankle_pitch", "right_knee", "right_hip_pitch",
                             "right_hip_roll", "right_hip_yaw", "right_elbow_pitch", "right_shoulder_twirl",
                             "right_shoulder_roll", "right_shoulder_pitch", "pelvis_yaw", "left_ankle_roll",
                             "left_ankle_pitch", "left_knee", "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
                             "left_elbow_pitch", "left_shoulder_twirl", "left_shoulder_roll",
                             "left_shoulder_pitch", "head_yaw", "head_pitch"]
        self.trims = [ 0,0,0,0, 0, 0, 0, 0, -0.12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.12, 0, 0, 0]

    def game_time(self):
        return self.robot.current_time/1000
        #while True:
        #    self.robot.add_to_queue(({},{}))
        #    self.robot.send_message()
        #    message = self.robot.client.receive()
        #    self.robot.update_history(message)
        #    time_s = self.robot.get_sensor("time")
        #    #time_s = self.robot.get_sensor("gps_body")
        #    if time_s: break
        #    #time.sleep(0.002)
        #return time_s['sim time']/1000
        ##return time_s['time']/1000

    def game_time_ms(self):
        return self.robot.current_time
        #while True:
        #    self.robot.add_to_queue(({},{}))
        #    self.robot.send_message()
        #    message = self.robot.client.receive()
        #    self.robot.update_history(message)
        #    time_ms = self.robot.get_sensor("time")
        #    #time_ms = self.robot.get_sensor("gps_body")
        #    if time_ms: break
        #    #time.sleep(0.002)
        #return time_ms['sim time']
        ##return time_ms['time']

    def pause_in_ms(self, time_in_ms):
        self.sim_Progress(time_in_ms/1000)

    def sim_Trigger(self):
        if not self.pause.Flag:
            if self.gcreceiver != None:
                if self.gcreceiver.team_state != None:
                    if self.gcreceiver.player_state.penalty != 0 or self.gcreceiver.state.game_state != 'STATE_PLAYING':
                        self.falling_Flag = 3
                        servo_data = {}
                        for key in self.WBservosList:
                            servo_data.update({key: 0})
                        self.robot.send_servos(servo_data)
            self.wait_for_step(self.timestep)

    def wait_for_step(self, step):
        while True:
            time1 = self.game_time_ms()
            if time1 >= (self.former_step_time + step):
                #print('simulation step time', time1 - self.former_step_time, 'real step time ', (time.time() - self.former_real_time)*1000)
                self.former_step_time = time1
                #self.former_real_time = time.time()
                break
            else:
                pass
                #time.sleep(0.002)


    def imu_activation(self):
        print("imu_activation")
        #while True:
        #    self.wait_for_step(15)
        #    print("imu_activation: imu request")
        #    self.robot.add_to_queue(({},{}))
        #    self.robot.send_message()
        #    message = self.robot.client.receive()
        #    self.robot.update_history(message)
        #    #print('message:', message)
        #    #head_euler = self.robot.get_sensor("imu_head")
        #    #body_euler = self.robot.get_sensor("imu_body")
        #    body_euler = robot.get_imu_body()
        #    #if head_euler and body_euler:
        #    if body_euler:
        #        #head_euler = head_euler['position']
        #        body_euler = body_euler['position']
        #        break
        #    else:
        #        print("imu_activation: imu does't answer")
        ##self.euler_angle['roll'] = head_euler[0]
        ##self.euler_angle['pitch'] = head_euler[1]
        ##self.euler_angle['yaw'] = head_euler[2]
        body_euler = self.robot.get_imu_body()['position']
        self.body_euler_angle['roll'] = body_euler[0]
        self.body_euler_angle['pitch'] = body_euler[1]
        self.body_euler_angle['yaw'] = body_euler[2]
        #print('head_yaw = ', head_euler[2], 'body_yaw =', body_euler[2])
        return self.body_euler_angle

    def read_head_imu_euler_angle(self):
        #while True:
        #    #self.wait_for_step(5)
        #    self.robot.add_to_queue(({},{}))
        #    self.robot.send_message()
        #    message = self.robot.client.receive()
        #    self.robot.update_history(message)
        #    head_euler = self.robot.get_sensor("imu_head")
        #    if head_euler:
        #        head_euler = head_euler['position']
        #        break
        #    else:
        #        print("read_head_imu_euler_angle: imu does't answer")
        head_euler = self.robot.get_imu_head()
        self.euler_angle['roll'] = head_euler[0]
        self.euler_angle['pitch'] = head_euler[1]
        self.euler_angle['yaw'] = head_euler[2]

    def read_imu_body_yaw(self):
        timer1 = time.perf_counter()
        #while True:
        #    #self.wait_for_step(5)
        #    #self.robot.add_to_queue(({},{}))
        #    #self.robot.send_message()
        #    #message = self.robot.client.receive()
        #    #self.robot.update_history(message)
        #    #body_euler = self.robot.get_sensor("imu_body")
        #    body_euler = self.robot.get_imu_body()
        #    if body_euler:
        #        #body_euler = body_euler['position']
        #        break
        #    else:
        #        print("read_imu_body_yaw: imu does't answer")
        #        #self.robot.add_to_queue(({},{}))
        #        #self.robot.send_message()
        body_euler = self.robot.get_imu_body()['position']
        self.body_euler_angle = {'roll': body_euler[0], 'pitch': body_euler[1], 'yaw': body_euler[2]}
        print('imu_body_measurement_time:', time.perf_counter() - timer1)
        return body_euler[2]

    def falling_Test(self):
        #if self.gcreceiver != None:
        #    if self.gcreceiver.team_state != None:
        #        if self.gcreceiver.state.game_state != 'STATE_PLAYING' or self.gcreceiver.player_state.penalty != 0:
        #            self.falling_Flag = 3
        #            self.simulateMotion(name = 'Initial_Pose')
        #            return self.falling_Flag
        #while True:
        #    #self.wait_for_step(5)
        #    self.robot.add_to_queue(({},{}))
        #    self.robot.send_message()
        #    message = self.robot.client.receive()
        #    self.robot.update_history(message)
        #    body_euler = self.robot.get_sensor("imu_body")
        #    if body_euler:
        #        body_euler = body_euler['position']
        #        #print('body_euler :', body_euler)
        #        break
        #    else:
        #        print("falling_Test: imu does't answer")
        #self.body_euler_angle['roll'], self.body_euler_angle['pitch'], self.body_euler_angle['yaw'] = body_euler
        #print('self.body_euler_angle[pitch] =', self.body_euler_angle['pitch'])
        #print('self.body_euler_angle[roll] =', self.body_euler_angle['roll'])
        #print('self.body_euler_angle[yaw] =', self.body_euler_angle['yaw'])
        if (self.body_euler_angle['pitch']) > 0.785:
            self.falling_Flag = 1                   # on stomach
            self.simulateMotion(name = 'Soccer_Get_UP_Stomach_N')
        if (self.body_euler_angle['pitch']) <  -0.785:
            self.falling_Flag = -1                  # face up
            self.simulateMotion(name = 'Soccer_Get_UP_Face_Up')
        if (self.body_euler_angle['roll']) > 0.785:
            self.falling_Flag = -2                  # on right side
            self.simulateMotion(name = 'Get_Up_Right')
        if -135< (self.body_euler_angle['roll']) < -0.785:
            self.falling_Flag = 2                   # on left side
            self.simulateMotion(name = 'Get_Up_Left')
        return self.falling_Flag

    def send_angles_to_servos(self, angles):
        self.sim_Trigger()
        time_ms = self.game_time_ms()
        print('time_step:', time_ms - self.sim_step_counter)
        self.sim_step_counter = time_ms
        servo_data = {}
        for i in range(len(angles)):
            key = self.WBservosList[i]
            value = angles[i] + self.trims[i]
            servo_data.update({key:value})
        #self.robot.add_to_queue((servo_data,{}))
        #self.robot.send_message()
        self.robot.send_servos(servo_data)
        #message = self.robot.client.receive()
        #self.robot.update_history(message)

        #self.sim_Trigger()
        #self.body_euler_angle['roll'], self.body_euler_angle['pitch'], self.body_euler_angle['yaw'] = self.robot.get_sensor("imu_body")['position']
        

    def move_head(self, pan, tilt):
        servo_data = {}
        pan_key = self.WBservosList[21]
        pan_value = pan * self.TIK2RAD + self.trims[21]
        tilt_key = self.WBservosList[22]
        tilt_value = tilt * self.TIK2RAD + self.trims[22]
        #self.robot.add_to_queue(({pan_key: pan_value, tilt_key: tilt_value}, {}))
        #self.robot.send_message()
        #message = self.robot.client.receive()
        #self.robot.update_history(message)
        servo_data = {pan_key: pan_value, tilt_key: tilt_value}
        self.robot.send_servos(servo_data)
        for i in range(16):
            self.sim_Trigger()

    def simulateMotion(self, number = 0, name = ''):
        #mot = [(0,'Initial_Pose'),(1,0),(2,0),(3,0),(4,0),(5,'Get_Up_Left'),
        #   (6,'Soccer_Get_UP_Stomach_N'),(7,0),(8,'Soccer_Walk_FF'),(9,0),(10,0),
        #   (11,0),(12,0),(13,0),(14,'Soccer_Small_Jump_Forward'),(15,0),
        #   (16,0),(17,0),(18,'Soccer_Kick_Forward_Right_Leg'),(19,'Soccer_Kick_Forward_Left_Leg'),(20,0),
        #   (21,'Get_Up_From_Defence'),(22,0),(23,'PanaltyDefenceReady_Fast'), (24,'PenaltyDefenceF'),(25,0),
        #   (26,0),(27,0),(28,0),(29.0),(30,'Soccer_Walk_FF0'),
        #   (31,'Soccer_Walk_FF1'), (32,'Soccer_Walk_FF2'), (33,'Soccer_Get_UP_Stomach'), (34,'Soccer_Get_UP_Face_Up'),
        #   (35,'Get_Up_Right'), (36,'PenaltyDefenceR'), (37,'PenaltyDefenceL')]
        # start the simulation
        if number > 0 and name == '': name = self.MOTION_SLOT_DICT[number]
        with open(self.glob.current_work_directory /"Soccer" / "Motion" / "motion_slots" / (name + ".json"), "r") as f:
            slots = json.loads(f.read())
        mot_list = slots[name]
        i=0
        for i in range(len(mot_list)):
            if  self.falling_Flag ==3: return
            activePoseOld = []
            for ind in range(len(self.activePose)): activePoseOld.append(self.activePose[ind])
            self.activePose =[]
            for j in range(len(self.ACTIVEJOINTS) - 2):
                    self.activePose.append(0.017*mot_list[i][j+1]*0.03375)
            pulseNum = int(mot_list[i][0]*self.FRAMELENGTH * 1000 / self.simThreadCycleInMs)
            for k in range (pulseNum):
                servo_data = {}
                for j in range(len(self.ACTIVEJOINTS) - 2):
                    tempActivePose = activePoseOld[j]+(self.activePose[j]-activePoseOld[j])*k/pulseNum
                    key = self.WBservosList[j]
                    value = tempActivePose + self.trims[j]
                    servo_data.update({key:value})
                #self.robot.add_to_queue((servo_data, {}))
                #self.robot.send_message()
                #message = self.robot.client.receive()
                #self.robot.update_history(message)
                self.robot.send_servos(servo_data)
                self.sim_Trigger()
        return

    def sim_Get_Ball_Position(self):
        #self.robot.add_to_queue(({},{"recognition": 5}))
        #self.robot.send_message()
        #message = self.robot.client.receive()
        #self.robot.update_history(message)
        #ball_pos = self.robot.get_sensor("BALL")
        #if ball_pos: 
        #    ball_position = ball_pos['position']
        #    print('ball_position =', ball_position)
        #    return ball_position
        ball_position = self.robot.get_ball()
        print('ball_position', ball_position)
        if ball_position:
            return ball_position
        else: return False

    def sim_Get_Obstacles(self):
        robot_names = ['RED_PLAYER_1', 'RED_PLAYER_2', 'BLUE_PLAYER_1', 'BLUE_PLAYER_2']
        my_name = self.robot.getSelf().getDef()
        robot_names.pop(robot_names.index(my_name))
        #print('my_name:', my_name)
        factor = self.local.side_factor
        new_obstacles = []
        ball_position = self.robot.getFromDef("BALL").getPosition()
        new_obstacles.append([factor*ball_position[0], factor*ball_position[1], 0.15] )
        for name in robot_names:
            obstacle = self.robot.getFromDef(name).getPosition()
            new_obstacles.append([factor*obstacle[0], factor*obstacle[1], 0.20])
        self.glob.obstacles = new_obstacles
        print ('new_obstacles:', new_obstacles)
        return

    def sim_Get_Robot_Position(self):
        self.sim_Trigger()
        #self.robot.add_to_queue(({},{}))
        #self.robot.send_message()
        #message = self.robot.client.receive()
        #self.robot.update_history(message)
        #Position = self.robot.get_sensor("gps_body")['position']
        Position = self.robot.get_localization()
        x, y  = Position['position']
        #self.body_euler_angle['roll'], self.body_euler_angle['pitch'], self.body_euler_angle['yaw'] = self.robot.get_sensor("imu_body")['position']
        self.body_euler_angle['roll'], self.body_euler_angle['pitch'], self.body_euler_angle['yaw'] = self.robot.get_imu_body()['position']
        print('Position: ', Position, 'yaw :', self.body_euler_angle['yaw'])
        self.body_euler_angle['yaw'] -= self.direction_To_Attack
        return x, y, self.body_euler_angle['yaw']

    def sim_Start(self):
        for i in range(len(self.ACTIVEJOINTS)):
            position = 0
            self.activePose.append(position)

    def sim_Progress(self, simTime):  # simTime in seconds
        timer1 = self.game_time_ms()
        while True:
            time.sleep(0.002)
            if simTime * 1000 + timer1 < self.game_time_ms(): break

if __name__=="__main__":
    print('This is not main module!')


