#  Walking engine for Starkit Kondo OpenMV
#  Copyright STARKIT Soccer team of MIPT

import sys, os
import math, time, json
from controller import *

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if current_work_directory.find('Soccer') >= 0:
    current_work_directory = current_work_directory[:-14]

current_work_directory += '/'
#with open(current_work_directory + "simulator_lib_directory.txt", "r") as f:
#    simulator_lib_directory = f.read()
#simulator_lib_directory = simulator_lib_directory.replace('\\', '/')
#sys.path.append(simulator_lib_directory)
import random
#import sim, threading

sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')
sys.path.append( current_work_directory + 'Soccer/Localisation/')



from class_Motion import *
from class_Motion_real import Motion_real
from compute_Alpha_v3 import Alpha

class Transfer_Data():
    def __init__(self):
        self.stop_Flag = False
        self.finish_Flag = False
        self.pause = False
        self.stop = 0
        self.finish = 0


#def sim_Enable(ip_address, port):
#    simThreadCycleInMs = 2
#    print ('Simulation started')
#    #sim.simxFinish(-1) # just in case, close all opened connections
#    clientID = sim.simxStart(ip_address, port, True, True, 5000, simThreadCycleInMs)
#    if clientID != -1:
#        print ('Connected to remote API server')
#    else:
#        print ('Failed connecting to remote API server')
#        print ('Program ended')
#        exit(0)
#    return clientID

#def simulation_Trigger_Accumulator(clientIDs, events, transfer_Datas, lock):
#    while(True):
#        finish = False
#        while(True):
#            for transfer_Data in transfer_Datas:
#                if transfer_Data.stop_Flag == True:
#                    finish = True
#                    for transfer_Data in transfer_Datas:
#                        if transfer_Data.finish_Flag == False: finish = False
#                    for transfer in transfer_Datas:
#                        transfer.stop_Flag = True
#                    break
#            event_flag = True
#            for i in range(len(events)):
#                if (not events[i].is_set()): event_flag = False
#                if transfer_Datas[i].finish_Flag == True:
#                    event_flag = True
#                    break
#            if event_flag == True: break
#            time.sleep(0.001)
#        lock.acquire()
#        lock.release()
#        for clientID in clientIDs:
#            sim.simxSynchronousTrigger(clientID)
#        for event in events: event.clear()
#        if finish == True:
#            for clientID in clientIDs:
#                sim.simxFinish(clientID)
#            break
#    pass


class Motion_sim(Motion_real):
    def __init__(self, glob):
        self.FRAMELENGTH = 0.02
        import random as random
        self.random = random
        #import sim as vr
        #self.sim = vr
        import numpy as np
        self.np = np
        import matplotlib.pyplot as plt
        self.plt = plt
        import cv2 as cv2
        self.cv2 = cv2
        import msvcrt as ms
        self.ms = ms
        import time
        self.utime = time
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
        #self.transfer_Data = transfer_Data
        #self.lock = lock
        #self.motion_Event = motion_EventID
        #self.clientID = clientID
        #self.robot_Number = robot_Number
        #self.Vision_Sensor_Display_On = True
        self.sim_step_counter = 0
        #self.numberOfRobots = numberOfRobots
        super().__init__(glob)
        with open(current_work_directory + "Init_params/Sim_calibr.json", "r") as f:
            data1 = json.loads(f.read())
        self.neck_calibr = data1['neck_calibr']
        self.neck_play_pose = data1['neck_play_pose']
        self.head_pitch_with_horizontal_camera = data1['head_pitch_with_horizontal_camera']
        self.neck_tilt = self.neck_calibr
        self.Vision_Sensor_Display_On = self.glob.params['Vision_Sensor_Display_On']
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
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
        self.FACTOR =  [ 1,-1,-1,-1, 1, 1, -1,1,1,-1,-1, 1,-1, -1,-1, 1, 1, -1,1,1,-1, 1, 1]
        self.trims = [ 0,0,0,0, 0, 0, 0, 0, -0.12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.12, 0, 0, 0]
        self.WBservos = {
                    (10,2) : "right_ankle_roll",
                    (9,2) : "right_ankle_pitch",
                    (8,2) : "right_knee",
                    (7,2) : "right_hip_pitch",
                    (6,2) : "right_hip_roll",
                    (5,2) : "right_hip_yaw",
                    (4, 2) : "right_elbow_pitch",
                    (3, 2) : "right_shoulder_twirl",
                    (2, 2) : "right_shoulder_roll",
                    (1, 2) : "right_shoulder_pitch",
                    (0, 2) : "pelvis_yaw",
                    (10,1) : "left_ankle_roll",
                    (9,1) : "left_ankle_pitch",
                    (8,1) : "left_knee",
                    (7,1) : "left_hip_pitch",
                    (6,1) : "left_hip_roll",
                    (5,1) : "left_hip_yaw",
                    (4,1) : "left_elbow_pitch",
                    (3,1) : "left_shoulder_twirl",
                    (2,1) : "left_shoulder_roll",
                    (1,1) : "left_shoulder_pitch",
                    (0,1) : "head_yaw",
                    (12,2): "head_pitch"
                  }

    def pause_in_ms(self, time_in_ms):
        self.sim_Progress(time_in_ms/1000)

    #def from_vrep_quat_to_conventional_quat(self, quaternion):
    #        x,y,z,w = quaternion
    #        return [w,x,y,z]

    def imu_activation(self):
        self.robot.getDevice("imu_head").enable(1)
        self.robot.getDevice("imu_body").enable(1)
        head_euler = self.robot.getDevice("imu_head").getRollPitchYaw()
        body_euler = self.robot.getDevice("imu_body").getRollPitchYaw()
        self.euler_angle['roll'] = head_euler[0]
        self.euler_angle['pitch'] = head_euler[1]
        self.euler_angle['yaw'] = head_euler[2]
        self.body_euler_angle['roll'] = body_euler[0]
        self.body_euler_angle['pitch'] = body_euler[1]
        self.body_euler_angle['yaw'] = body_euler[2]
        #time.sleep(0.1)
        #if self.glob.SIMULATION != 0:
        #    self.sim.simxStartSimulation(self.clientID,self.sim.simx_opmode_oneshot)
        #returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
        #Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
        #self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
        #self.read_head_imu_euler_angle()
        ##print('body_euler_angle = ', self.body_euler_angle)
        #returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
        #self.Dummy_HData.append(Dummy_Hposition)
        #returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
        return self.euler_angle

    def read_head_imu_euler_angle(self):
        head_euler = self.robot.getDevice("imu_head").getRollPitchYaw()
        self.euler_angle['roll'] = head_euler[0]
        self.euler_angle['pitch'] = head_euler[1]
        self.euler_angle['yaw'] = head_euler[2]
        print('self.euler_angle[yaw] =', self.euler_angle['yaw'])
        #returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
        #Dummy_Hquaternion = self.from_vrep_quat_to_conventional_quat(Dummy_Hquaternion)
        #self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)

    def falling_Test(self):
        key = 0
        if self.ms.kbhit():
            key = self.ms.getch()
        if key == b'p' :
            self.lock.acquire()
            if self.glob.SIMULATION == 3:
                #self.sim.simxPauseSimulation(self.clientID, self.sim.simx_opmode_oneshot)
                pass
            key = 0
            while (True):
                if self.ms.kbhit():
                    key = self.ms.getch()
                if key == b'p':
                    self.lock.release()
                    if self.glob.SIMULATION == 3:
                        #self.sim.simxStartSimulation(self.clientID, self.sim.simx_opmode_oneshot)
                        pass
                    key = 0
                    break
        if key == b's' :
            print('Simulation STOP by keyboard')
            self.sim_Stop()
            self.falling_Flag = 3
            return self.falling_Flag
        #returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
        #Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
        #self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
        self.body_euler_angle['roll'], self.body_euler_angle['pitch'], self.body_euler_angle['yaw'] = self.robot.getDevice("imu_body").getRollPitchYaw()
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
        for i in range(len(angles)):
            self.robot.getDevice(self.WBservosList[i]).setPosition(angles[i] * self.FACTOR[i] + self.trims[i])
        self.body_euler_angle['roll'], self.body_euler_angle['pitch'], self.body_euler_angle['yaw'] = self.robot.getDevice("imu_body").getRollPitchYaw()
        self.robot.step(20)
        #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
        #    if self.glob.SIMULATION == 3: self.wait_sim_step()
        #    for i in range(len(angles)):
        #        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
        #            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
        #                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
        #                        self.sim.simx_opmode_oneshot)
        #        elif self.glob.SIMULATION == 0:
        #            returnCode = self.sim.simxSetJointPosition(self.clientID,
        #                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
        #                        self.sim.simx_opmode_oneshot)
        #    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
        #        time.sleep(self.slowTime)
        #        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
        #        self.Dummy_HData.append(Dummy_Hposition)
        #        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
        #        self.BallData.append(self.Ballposition)
        #        returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
        #        Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
        #        self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
        #        #print(self.euler_angle)
        #        self.timeElapsed = self.timeElapsed +1
        #        #print(Dummy_Hposition)
        #        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
        #            self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
        #        if self.glob.SIMULATION == 1:
        #            self.sim_simxSynchronousTrigger(self.clientID)

    def move_head(self, pan, tilt):
        self.robot.getDevice(self.WBservosList[21]).setPosition(pan * self.TIK2RAD * self.FACTOR[21] + self.trims[21])
        self.robot.getDevice(self.WBservosList[22]).setPosition(tilt * self.TIK2RAD * self.FACTOR[22] + self.trims[22])
        #returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
        #            self.jointHandle[21] , pan * self.TIK2RAD * self.FACTOR[21], self.sim.simx_opmode_oneshot)   # Шея поворот
        #returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
        #            self.jointHandle[22] , tilt * self.TIK2RAD * self.FACTOR[22], self.sim.simx_opmode_oneshot)  # Шея Наклон
        #for j in range(20):
        #    self.sim_simxSynchronousTrigger(self.clientID)

    #def wait_sim_step(self):
    #    while True:
    #        self.sim.simxGetIntegerParameter(self.clientID, self.sim.sim_intparam_program_version, self.sim.simx_opmode_buffer)
    #        tim = self.sim.simxGetLastCmdTime(self.clientID)
    #        #print ('Simulation time: ', tim)
    #        if tim > self.sim_step_counter:
    #            self.sim_step_counter = tim 
    #            break
    #        time.sleep(0.004)
    #        if self.transfer_Data.stop > 0:
    #            self.transfer_Data.stop += 1
    #            self.sim_Stop()
    #            while True:
    #                if self.transfer_Data.stop == self.numberOfRobots:
    #                    self.sim_Disable()
    #                    sys.exit(0)
    #                time.sleep(0.1)

    #def getSimTime(self):
    #    inputInts=[]
    #    inputFloats=[]
    #    inputStrings=[]
    #    inputBuffer=bytearray()
    #    name = 'Telo_Surrogat'+ self.robot_Number
    #    self.sim.simxPauseSimulation(self.clientID, self.sim.simx_opmode_oneshot)
    #    res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(self.clientID,name,sim.sim_scripttype_childscript,
    #                    'getSimTime',inputInts,inputFloats,inputStrings,inputBuffer,sim.simx_opmode_blocking)
    #    self.sim.simxStartSimulation(self.clientID, self.sim.simx_opmode_oneshot)   
    #    if len(retFloats) == 0: return 0
    #    return retFloats[0]

    #def sim_simxSynchronousTrigger(self, clientID):
    #    if  self.glob.SIMULATION == 1 :
    #        if self.transfer_Data.stop > 0:
    #                self.transfer_Data.stop += 1
    #                self.sim_Stop()
    #                while True:
    #                    if self.transfer_Data.stop == self.numberOfRobots:
    #                        self.sim_Disable()
    #                        sys.exit(0)
    #                    time.sleep(0.1)
    #    if  self.glob.SIMULATION == 3 : 
    #        self.wait_sim_step()
    #        return
    #    if self.motion_Event == 0:
    #        self.sim.simxSynchronousTrigger(clientID)
    #    else:
    #        self.motion_Event.set()
    #        while (self.motion_Event.is_set()): time.sleep(0.001)

    def vision_Sensor_Get_Image(self):
        if self.Vision_Sensor_Display_On:
            #returnCode, resolution, image_Data = self.sim.simxGetVisionSensorImage(self.clientID, self.VisionHandle, 0 ,self.sim.simx_opmode_buffer)
            nuimg = self.np.array(image_Data, dtype=self.np.uint8)
            nuimg.shape = (resolution[1],resolution[0],3)
            nuimg1 = self.cv2.cvtColor(nuimg, self.cv2.COLOR_RGB2BGR)
            img = self.np.flip(nuimg1, 1)
            return img

    def vision_Sensor_Display(self, img):
        if self.Vision_Sensor_Display_On:
            self.cv2.imshow('Vision Sensor'+ self.robot_Number, img)
            self.cv2.waitKey(10) & 0xFF
            #if self.robot_Number != '':
            #    self.cv2.waitKey(10) & 0xFF
            #else:
            #    res = self.cv2.waitKey(0)
            #    if res == 115:
            #        print('you have pressed "s"')
            #        token = str(int(self.random.random()*10000))
            #        filename = current_work_directory + "Soccer/CameraStill/VisionSensor" + token + '.png'
            #        isWritten = self.cv2.imwrite(filename, img)

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
        with open(current_work_directory + "Soccer/Motion/motion_slots/" + name + ".json", "r") as f:
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
                #if self.glob.SIMULATION == 3: self.wait_sim_step()
                #self.sim.simxPauseCommunication(self.clientID, True)
                for j in range(len(self.ACTIVEJOINTS) - 2):
                    tempActivePose = activePoseOld[j]+(self.activePose[j]-activePoseOld[j])*k/pulseNum
                    self.robot.getDevice(self.WBservosList[j]).setPosition(tempActivePose * self.FACTOR[j] + self.trims[j])
                    #returnCode = self.sim.simxSetJointTargetPosition(self.clientID, self.jointHandle[j] ,
                    #             tempActivePose*self.FACTOR[j] +self.trims[j], self.sim.simx_opmode_streaming)
                #self.sim.simxPauseCommunication(self.clientID, False)
                self.robot.step(20)
                #if self.glob.SIMULATION == 1:
                #    self.sim_simxSynchronousTrigger(self.clientID)
        return

    def sim_Get_Ball_Position(self):
        #returnCode, Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
        return self.robot.getFromDef("ball").getPosition()

    def sim_Get_Robot_Position(self):
        x, y, z  = self.robot.getSelf().getPosition()
        self.body_euler_angle['roll'], self.body_euler_angle['pitch'], self.body_euler_angle['yaw'] = self.robot.getDevice("imu_body").getRollPitchYaw()
        return x, y, self.body_euler_angle['yaw']
        #returnCode, Dummy_1position= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
        #returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
        #Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
        #euler_angles = self.quaternion_to_euler_angle(Dummy_1quaternion)
        #return Dummy_1position[0], Dummy_1position[1], euler_angles['yaw']


    def sim_Start(self):
        for i in range(len(self.ACTIVEJOINTS)):
            position = self.robot.getDevice(self.WBservosList[i]).getTargetPosition()
            self.activePose.append(position)
        pass
        ##print ('Simulation started')
        #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
        #    #self.sim.simxFinish(-1) # just in case, close all opened connections
        #    #self.clientID=self.sim.simxStart('127.0.0.1',19997,True,True,5000,self.simThreadCycleInMs) # Connect to V-REP
        #    #if self.clientID!=-1:
        #    #    print ('Connected to remote API server')
        #    #else:
        #    #    print ('Failed connecting to remote API server')
        #    #    print ('Program ended')
        #    #    exit(0)
        #    ## Collect Joint Handles and trims from model
        #    returnCode, self.Dummy_HHandle = self.sim.simxGetObjectHandle(self.clientID, 'Dummy_H'+ self.robot_Number, self.sim.simx_opmode_blocking)
        #    returnCode, self.Dummy_1Handle = self.sim.simxGetObjectHandle(self.clientID, 'Dummy1' + self.robot_Number, self.sim.simx_opmode_blocking)
        #    returnCode, self.BallHandle = self.sim.simxGetObjectHandle(self.clientID, 'Ball', self.sim.simx_opmode_blocking)
        #    returnCode, self.VisionHandle = self.sim.simxGetObjectHandle(self.clientID, 'Vision_sensor' + self.robot_Number, self.sim.simx_opmode_blocking)
        #    returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_streaming)
        #    returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_streaming)
        #    returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_streaming)
        #    returnCode, Dummy_1position= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_streaming)
        #    returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_streaming)
        #    if self.Vision_Sensor_Display_On:
        #        returnCode, resolution, image_Data = self.sim.simxGetVisionSensorImage(self.clientID, self.VisionHandle, 0 ,self.sim.simx_opmode_streaming)
        #    returnCode, Camera_quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.VisionHandle , -1, self.sim.simx_opmode_streaming)
        #    #print(Dummy_Hquaternion)
        #    for i in range(len(self.ACTIVEJOINTS)):
        #        returnCode, handle= self.sim.simxGetObjectHandle(self.clientID, self.ACTIVEJOINTS[i] + self.robot_Number, self.sim.simx_opmode_blocking)
        #        self.jointHandle.append(handle)
        #        returnCode, position= self.sim.simxGetJointPosition(self.clientID, handle, self.sim.simx_opmode_blocking)
        #        self.trims.append(position)
        #        self.activePose.append(position)
        #    if self.glob.SIMULATION == 1:
        #        self.sim.simxSynchronous(self.clientID,True)
        #    if self.glob.SIMULATION == 3:
        #        self.sim.simxGetIntegerParameter(self.clientID, self.sim.sim_intparam_program_version, self.sim.simx_opmode_streaming)

    def sim_Progress(self,simTime):  # simTime in seconds
        for i in range(int(simTime*1000//self.timestep)):
            self.robot.step(20)
        #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
        #    for i in range(int(simTime*1000//self.simThreadCycleInMs)):
        #        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
        #        self.Dummy_HData.append(Dummy_Hposition)
        #        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
        #        self.BallData.append(self.Ballposition)
        #        returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
        #        #print(quaternion_to_euler_angle(Dummy_Hquaternion))
        #        self.timeElapsed = self.timeElapsed +1
        #        self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
        #        if self.glob.SIMULATION == 1 : self.sim_simxSynchronousTrigger(self.clientID)
        #        if self.glob.SIMULATION == 3 : 
        #            time.sleep(0.005)
        #            self.wait_sim_step() 

    def sim_Stop(self):
        pass
        #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
        #    self.sim.simxStopSimulation(self.clientID,self.sim.simx_opmode_oneshot)
        #            # return to initial pose
        #    for j in range(len(self.ACTIVEJOINTS)):
        #        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
        #           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
        #                        self.jointHandle[j] , self.trims[j], self.sim.simx_opmode_oneshot)
        #        else: returnCode = self.sim.simxSetJointPosition(self.clientID,
        #                           self.jointHandle[j] , self.trims[j], self.sim.simx_opmode_oneshot)

    def sim_Disable(self):            # Now close the connection to V-REP:
        pass
        #time.sleep(0.2)
        ##self.sim.simxFinish(self.clientID)
        #self.transfer_Data.finish_Flag = True


if __name__=="__main__":
    print('This is not main module!')


