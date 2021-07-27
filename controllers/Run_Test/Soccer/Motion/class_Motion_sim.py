#  Walking engine for Starkit Kondo OpenMV
#  Copyright STARKIT Soccer team of MIPT

import sys, os
import math, time, json

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if current_work_directory.find('Soccer') >= 0:
    current_work_directory = current_work_directory[:-14]

current_work_directory += '/'
with open(current_work_directory + "simulator_lib_directory.txt", "r") as f:
    simulator_lib_directory = f.read()
simulator_lib_directory = simulator_lib_directory.replace('\\', '/')
sys.path.append(simulator_lib_directory)
import random
import sim, threading

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


def sim_Enable(ip_address, port):
    simThreadCycleInMs = 2
    print ('Simulation started')
    #sim.simxFinish(-1) # just in case, close all opened connections
    clientID = sim.simxStart(ip_address, port, True, True, 5000, simThreadCycleInMs)
    if clientID != -1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')
        print ('Program ended')
        exit(0)
    return clientID

def simulation_Trigger_Accumulator(clientIDs, events, transfer_Datas, lock):
    while(True):
        finish = False
        while(True):
            for transfer_Data in transfer_Datas:
                if transfer_Data.stop_Flag == True:
                    finish = True
                    for transfer_Data in transfer_Datas:
                        if transfer_Data.finish_Flag == False: finish = False
                    for transfer in transfer_Datas:
                        transfer.stop_Flag = True
                    break
            event_flag = True
            for i in range(len(events)):
                if (not events[i].is_set()): event_flag = False
                if transfer_Datas[i].finish_Flag == True:
                    event_flag = True
                    break
            if event_flag == True: break
            time.sleep(0.001)
        lock.acquire()
        lock.release()
        for clientID in clientIDs:
            sim.simxSynchronousTrigger(clientID)
        for event in events: event.clear()
        if finish == True:
            for clientID in clientIDs:
                sim.simxFinish(clientID)
            break
    pass


class Motion_sim(Motion_real):
    def __init__(self, glob, clientID , motion_EventID,  lock, transfer_Data, numberOfRobots, robot_Number = ''):
        self.FRAMELENGTH = 0.02
        import random as random
        self.random = random
        import sim as vr
        self.sim = vr
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
        self.transfer_Data = transfer_Data
        self.lock = lock
        self.motion_Event = motion_EventID
        self.clientID = clientID
        self.robot_Number = robot_Number
        #self.Vision_Sensor_Display_On = True
        self.sim_step_counter = 0
        self.numberOfRobots = numberOfRobots
        super().__init__(glob)
        with open(current_work_directory + "Init_params/Sim_calibr.json", "r") as f:
            data1 = json.loads(f.read())
        self.neck_calibr = data1['neck_calibr']
        self.neck_play_pose = data1['neck_play_pose']
        self.head_pitch_with_horizontal_camera = data1['head_pitch_with_horizontal_camera']
        self.neck_tilt = self.neck_calibr
        self.Vision_Sensor_Display_On = self.glob.params['Vision_Sensor_Display_On']

    def pause_in_ms(self, time_in_ms):
        self.sim_Progress(time_in_ms/1000)

    def from_vrep_quat_to_conventional_quat(self, quaternion):
            x,y,z,w = quaternion
            return [w,x,y,z]

    def imu_activation(self):
        time.sleep(0.1)
        if self.glob.SIMULATION != 0:
            self.sim.simxStartSimulation(self.clientID,self.sim.simx_opmode_oneshot)
        returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
        Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
        self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
        self.read_head_imu_euler_angle()
        #print('body_euler_angle = ', self.body_euler_angle)
        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
        self.Dummy_HData.append(Dummy_Hposition)
        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
        return self.euler_angle

    def read_head_imu_euler_angle(self):
        returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
        Dummy_Hquaternion = self.from_vrep_quat_to_conventional_quat(Dummy_Hquaternion)
        self.euler_angle = self.quaternion_to_euler_angle(Dummy_Hquaternion)

    def falling_Test(self):
        if self.glob.SIMULATION == 0 or self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
            key = 0
            if self.ms.kbhit():
                key = self.ms.getch()
            if key == b'p' :
                self.lock.acquire()
                if self.glob.SIMULATION == 3:
                    self.sim.simxPauseSimulation(self.clientID, self.sim.simx_opmode_oneshot)
                key = 0
                while (True):
                    if self.ms.kbhit():
                        key = self.ms.getch()
                    if key == b'p':
                        self.lock.release()
                        if self.glob.SIMULATION == 3:
                            self.sim.simxStartSimulation(self.clientID, self.sim.simx_opmode_oneshot)
                        key = 0
                        break
            if key == b's' or self.transfer_Data.stop_Flag:
                print('Simulation STOP by keyboard')
                self.transfer_Data.stop += 1
                self.sim_Stop()
                while True:
                    if self.transfer_Data.stop == self.numberOfRobots:
                        self.sim_Disable()
                        sys.exit(0)
                    time.sleep(0.1)
                self.falling_Flag = 3
                self.transfer_Data.stop_Flag = True
                return self.falling_Flag
            returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
            Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
            self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
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
        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
            if self.glob.SIMULATION == 3: self.wait_sim_step()
            #self.sim.simxPauseCommunication(self.clientID, True)
            for i in range(len(angles)):
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                    returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                self.sim.simx_opmode_oneshot)
                elif self.glob.SIMULATION == 0:
                    returnCode = self.sim.simxSetJointPosition(self.clientID,
                                self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                self.sim.simx_opmode_oneshot)
            #self.sim.simxPauseCommunication(self.clientID, False)
            if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                time.sleep(self.slowTime)
                returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                self.Dummy_HData.append(Dummy_Hposition)
                returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                self.BallData.append(self.Ballposition)
                returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
                Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
                self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
                #print(self.euler_angle)
                self.timeElapsed = self.timeElapsed +1
                #print(Dummy_Hposition)
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                    self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                if self.glob.SIMULATION == 1:
                    self.sim_simxSynchronousTrigger(self.clientID)

    def move_head(self, pan, tilt):
        returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                    self.jointHandle[21] , pan * self.TIK2RAD * self.FACTOR[21], self.sim.simx_opmode_oneshot)   # Шея поворот
        returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                    self.jointHandle[22] , tilt * self.TIK2RAD * self.FACTOR[22], self.sim.simx_opmode_oneshot)  # Шея Наклон
        for j in range(20):
            self.sim_simxSynchronousTrigger(self.clientID)

    def wait_sim_step(self):
        while True:
            self.sim.simxGetIntegerParameter(self.clientID, self.sim.sim_intparam_program_version, self.sim.simx_opmode_buffer)
            tim = self.sim.simxGetLastCmdTime(self.clientID)
            #print ('Simulation time: ', tim)
            if tim > self.sim_step_counter:
                self.sim_step_counter = tim 
                break
            time.sleep(0.004)
            if self.transfer_Data.stop > 0:
                self.transfer_Data.stop += 1
                self.sim_Stop()
                while True:
                    if self.transfer_Data.stop == self.numberOfRobots:
                        self.sim_Disable()
                        sys.exit(0)
                    time.sleep(0.1)

    def getSimTime(self):
        inputInts=[]
        inputFloats=[]
        inputStrings=[]
        inputBuffer=bytearray()
        name = 'Telo_Surrogat'+ self.robot_Number
        self.sim.simxPauseSimulation(self.clientID, self.sim.simx_opmode_oneshot)
        res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(self.clientID,name,sim.sim_scripttype_childscript,
                        'getSimTime',inputInts,inputFloats,inputStrings,inputBuffer,sim.simx_opmode_blocking)
        self.sim.simxStartSimulation(self.clientID, self.sim.simx_opmode_oneshot)   
        if len(retFloats) == 0: return 0
        return retFloats[0]

    def sim_simxSynchronousTrigger(self, clientID):
        if  self.glob.SIMULATION == 1 :
            if self.transfer_Data.stop > 0:
                    self.transfer_Data.stop += 1
                    self.sim_Stop()
                    while True:
                        if self.transfer_Data.stop == self.numberOfRobots:
                            self.sim_Disable()
                            sys.exit(0)
                        time.sleep(0.1)
        if  self.glob.SIMULATION == 3 : 
            self.wait_sim_step()
            return
        if self.motion_Event == 0:
            self.sim.simxSynchronousTrigger(clientID)
        else:
            self.motion_Event.set()
            while (self.motion_Event.is_set()): time.sleep(0.001)

    def vision_Sensor_Get_Image(self):
        if self.Vision_Sensor_Display_On:
            returnCode, resolution, image_Data = self.sim.simxGetVisionSensorImage(self.clientID, self.VisionHandle, 0 ,self.sim.simx_opmode_buffer)
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
                if self.glob.SIMULATION == 3: self.wait_sim_step()
                self.sim.simxPauseCommunication(self.clientID, True)
                for j in range(len(self.ACTIVEJOINTS) - 2):
                    tempActivePose = activePoseOld[j]+(self.activePose[j]-activePoseOld[j])*k/pulseNum
                    returnCode = self.sim.simxSetJointTargetPosition(self.clientID, self.jointHandle[j] ,
                                 tempActivePose*self.FACTOR[j] +self.trims[j], self.sim.simx_opmode_streaming)
                self.sim.simxPauseCommunication(self.clientID, False)
                if self.glob.SIMULATION == 1:
                    self.sim_simxSynchronousTrigger(self.clientID)
        return

    def sim_Get_Ball_Position(self):
        returnCode, Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
        return Ballposition

    def sim_Get_Robot_Position(self):
        returnCode, Dummy_1position= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
        returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
        Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
        euler_angles = self.quaternion_to_euler_angle(Dummy_1quaternion)
        return Dummy_1position[0], Dummy_1position[1], euler_angles['yaw']

    def sim_Start(self):
        #print ('Simulation started')
        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
            #self.sim.simxFinish(-1) # just in case, close all opened connections
            #self.clientID=self.sim.simxStart('127.0.0.1',19997,True,True,5000,self.simThreadCycleInMs) # Connect to V-REP
            #if self.clientID!=-1:
            #    print ('Connected to remote API server')
            #else:
            #    print ('Failed connecting to remote API server')
            #    print ('Program ended')
            #    exit(0)
            ## Collect Joint Handles and trims from model
            returnCode, self.Dummy_HHandle = self.sim.simxGetObjectHandle(self.clientID, 'Dummy_H'+ self.robot_Number, self.sim.simx_opmode_blocking)
            returnCode, self.Dummy_1Handle = self.sim.simxGetObjectHandle(self.clientID, 'Dummy1' + self.robot_Number, self.sim.simx_opmode_blocking)
            returnCode, self.BallHandle = self.sim.simxGetObjectHandle(self.clientID, 'Ball', self.sim.simx_opmode_blocking)
            returnCode, self.VisionHandle = self.sim.simxGetObjectHandle(self.clientID, 'Vision_sensor' + self.robot_Number, self.sim.simx_opmode_blocking)
            returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_streaming)
            returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_streaming)
            returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_streaming)
            returnCode, Dummy_1position= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_streaming)
            returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_streaming)
            if self.Vision_Sensor_Display_On:
                returnCode, resolution, image_Data = self.sim.simxGetVisionSensorImage(self.clientID, self.VisionHandle, 0 ,self.sim.simx_opmode_streaming)
            returnCode, Camera_quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.VisionHandle , -1, self.sim.simx_opmode_streaming)
            #print(Dummy_Hquaternion)
            for i in range(len(self.ACTIVEJOINTS)):
                returnCode, handle= self.sim.simxGetObjectHandle(self.clientID, self.ACTIVEJOINTS[i] + self.robot_Number, self.sim.simx_opmode_blocking)
                self.jointHandle.append(handle)
                returnCode, position= self.sim.simxGetJointPosition(self.clientID, handle, self.sim.simx_opmode_blocking)
                self.trims.append(position)
                self.activePose.append(position)
            if self.glob.SIMULATION == 1:
                self.sim.simxSynchronous(self.clientID,True)
            if self.glob.SIMULATION == 3:
                self.sim.simxGetIntegerParameter(self.clientID, self.sim.sim_intparam_program_version, self.sim.simx_opmode_streaming)

    def sim_Progress(self,simTime):  # simTime in seconds
        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
            for i in range(int(simTime*1000//self.simThreadCycleInMs)):
                returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                self.Dummy_HData.append(Dummy_Hposition)
                returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                self.BallData.append(self.Ballposition)
                returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                #print(quaternion_to_euler_angle(Dummy_Hquaternion))
                self.timeElapsed = self.timeElapsed +1
                self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                if self.glob.SIMULATION == 1 : self.sim_simxSynchronousTrigger(self.clientID)
                if self.glob.SIMULATION == 3 : 
                    time.sleep(0.005)
                    self.wait_sim_step() 

    def sim_Stop(self):
        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
            self.sim.simxStopSimulation(self.clientID,self.sim.simx_opmode_oneshot)
                    # return to initial pose
            for j in range(len(self.ACTIVEJOINTS)):
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                   returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                self.jointHandle[j] , self.trims[j], self.sim.simx_opmode_oneshot)
                else: returnCode = self.sim.simxSetJointPosition(self.clientID,
                                   self.jointHandle[j] , self.trims[j], self.sim.simx_opmode_oneshot)

    def sim_Disable(self):            # Now close the connection to V-REP:
        time.sleep(0.2)
        #self.sim.simxFinish(self.clientID)
        self.transfer_Data.finish_Flag = True


if __name__=="__main__":
    print('This is not main module!')


