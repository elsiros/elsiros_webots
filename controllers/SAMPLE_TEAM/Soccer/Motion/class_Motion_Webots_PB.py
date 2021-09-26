"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of A. Babaev.
The module is designed to provide communication from motion controller to simulation
"""
import sys, os
import math, time, json
import logging
import random
from .class_Motion import *
from .class_Motion_real import Motion_real
from .compute_Alpha_v3 import Alpha

class Motion_sim(Motion_real):
    def __init__(self, glob, robot, gcreceiver, pause, logger):
        self.logger = logger
        self.pause = pause
        self.FRAMELENGTH = 0.02
        import random as random
        self.random = random
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
        self.initial_time_for_chain = 0
        self.last_step_time = 0
        self.chain_step_number = 0
        super().__init__(glob)
        with open(self.glob.current_work_directory / "Init_params" / "Sim_calibr.json", "r") as f:
            data1 = json.loads(f.read())
        self.neck_calibr = data1['neck_calibr']
        self.neck_play_pose = data1['neck_play_pose']
        self.head_pitch_with_horizontal_camera = data1['head_pitch_with_horizontal_camera']
        self.neck_tilt = self.neck_calibr
        self.Vision_Sensor_Display_On = self.glob.params['Vision_Sensor_Display_On']
        self.timestep = 25  
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

    def game_time_ms(self):
        return self.robot.current_time

    def pause_in_ms(self, time_in_ms):
        self.sim_Progress(time_in_ms/1000)

    def sim_Trigger(self, time):
        if not self.pause.Flag:
            if self.gcreceiver != None:
                if self.gcreceiver.team_state != None:
                    if self.gcreceiver.player_state.penalty != 0 or self.gcreceiver.state.game_state != 'STATE_PLAYING':
                        self.falling_Flag = 3
                        servo_data = {}
                        for key in self.WBservosList:
                            servo_data.update({key: 0})
                        self.robot.send_servos(servo_data)
            self.wait_for_step(time)

    def wait_for_step(self, step):
        while True:
            time1 = self.game_time_ms()
            if time1 >= (self.former_step_time + step):
                self.former_step_time = time1
                break

    def imu_activation(self):
        self.logger.info("imu_activation")
        while True:
            body_euler = self.robot.get_imu_body()
            if body_euler: 
                body_euler = body_euler['position']
                break
        self.body_euler_angle['roll'] = body_euler[0]
        self.body_euler_angle['pitch'] = body_euler[1]
        self.body_euler_angle['yaw'] = body_euler[2]
        #print('head_yaw = ', head_euler[2], 'body_yaw =', body_euler[2])
        return self.body_euler_angle

    def read_head_imu_euler_angle(self):
        head_euler = self.robot.get_imu_head()
        self.euler_angle['roll'] = head_euler[0]
        self.euler_angle['pitch'] = head_euler[1]
        self.euler_angle['yaw'] = head_euler[2]

    def read_imu_body_yaw(self):
        #timer1 = time.perf_counter()
        body_euler = self.robot.get_imu_body()['position']
        self.body_euler_angle = {'roll': body_euler[0], 'pitch': body_euler[1], 'yaw': body_euler[2]}
        self.logger.debug('imu_body: '+ str(self.body_euler_angle))
        return body_euler[2]

    def falling_Test(self):
        if self.gcreceiver != None:
            if self.gcreceiver.team_state != None:
                if self.gcreceiver.state.game_state != 'STATE_PLAYING' or self.gcreceiver.player_state.penalty != 0:
                    self.falling_Flag = 3
                    self.simulateMotion(name = 'Initial_Pose')
                    self.logger.info('STOP!')
                    return self.falling_Flag
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
        if self.falling_Flag != 0: self.logger.info('FALLING!!!'+ str(self.falling_Flag))
        return self.falling_Flag

    def send_angles_to_servos(self, angles, use_step_correction = False):
        if use_step_correction:
            self.chain_step_number += 1
            target_time_for_chain = self.initial_time_for_chain + self.chain_step_number * self.timestep
            target_step_time = target_time_for_chain - self.robot.current_time
            if target_step_time < 0: target_step_time = 0
            self.sim_Trigger(target_step_time)
        else: 
            self.initial_time_for_chain = self.robot.current_time
            self.chain_step_number = 0
            self.sim_Trigger(self.timestep)
        servo_data = {}
        for i in range(len(angles)):
            key = self.WBservosList[i]
            value = angles[i] + self.trims[i]
            servo_data.update({key:value})
        self.robot.send_servos(servo_data)

    def move_head(self, pan, tilt):
        servo_data = {}
        pan_key = self.WBservosList[21]
        pan_value = pan * self.TIK2RAD + self.trims[21]
        tilt_key = self.WBservosList[22]
        tilt_value = tilt * self.TIK2RAD + self.trims[22]
        servo_data = {pan_key: pan_value, tilt_key: tilt_value}
        self.robot.send_servos(servo_data)
        for i in range(1):
            self.sim_Trigger(self.timestep)

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
        self.logger.info('simulate motion slot:'+ str(name))
        self.chain_step_number = 0
        self.initial_time_for_chain = self.robot.current_time
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
                angles = []
                for j in range(len(self.ACTIVEJOINTS) - 2):
                    tempActivePose = activePoseOld[j]+(self.activePose[j]-activePoseOld[j])*k/pulseNum
                    key = self.WBservosList[j]
                    value = tempActivePose + self.trims[j]
                    angles.append(value)
                    servo_data.update({key:value})
                self.send_angles_to_servos(angles, use_step_correction = True)
                #self.sim_Trigger(self.timestep)
        return

    def sim_Get_Ball_Position(self):
        ball_position = self.robot.get_ball()
        self.logger.debug('ball_position'+ str(ball_position))
        if ball_position:
            return ball_position["position"]
        else: return False

    def sim_Get_Obstacles(self):
        obstacle1 = self.robot.get_mates()
        try:
            obstacle1 = list(obstacle1['position'])
        except Exception:
            obstacle1 = []
        opponets = self.robot.get_opponents()
        try:
            obstacle2 = list(opponets[0]['position'])
        except Exception:
            obstacle2 = []
        try:
            obstacle3 = list(opponets[1]['position'])
        except Exception:
            obstacle3 = []
        self.logger.debug('measurements: obstacle1:'+ str(obstacle1) + ' obstacle2:'+ str(obstacle2) + ' obstacle3:'+ str(obstacle3))
        if obstacle1:
            x = obstacle1[1]* math.cos(obstacle1[0])
            y = obstacle1[1]* math.sin(obstacle1[0])
            self.glob.obstacles.append([x, y, 0.2])
        if obstacle2: 
            x = obstacle2[1]* math.cos(obstacle2[0])
            y = obstacle2[1]* math.sin(obstacle2[0])
            self.glob.obstacles.append([x, y, 0.2])
        if obstacle3: 
            x = obstacle3[1]* math.cos(obstacle3[0])
            y = obstacle3[1]* math.sin(obstacle3[0])
            self.glob.obstacles.append([x, y, 0.2])
        return

    def sim_Get_Robot_Position(self):
        self.sim_Trigger(self.timestep)
        Position = self.robot.get_localization()
        x, y  = Position['position']
        #self.body_euler_angle['roll'], self.body_euler_angle['pitch'], self.body_euler_angle['yaw'] = self.robot.get_sensor("imu_body")['position']
        self.body_euler_angle['roll'], self.body_euler_angle['pitch'], self.body_euler_angle['yaw'] = self.robot.get_imu_body()['position']
        self.logger.debug('Position: '+ str(Position) + ' yaw :' + str(self.body_euler_angle['yaw']))
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


