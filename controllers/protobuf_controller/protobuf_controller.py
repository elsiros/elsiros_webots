from controller import *
import servos_msg2_pb2_pb2
import sys
import math, time
import socket

from compute_Alpha_v3 import Alpha
maxServAngle = 4.712

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

address = ('127.0.0.1', 6006)
socket_pos = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket_pos.connect(address)
#socket_pos.bind(address)
#socket_pos = socket.create_connection(address)
#для получения информации о мире robot->supervisor


servos = {
    (10,2) : "left_ankle_roll",
    (9,2) : "left_ankle_pitch",
    (8,2) : "left_knee",
    (7,2) : "left_hip_pitch",
    (6,2) : "left_hip_roll",
    (5,2) : "left_hip_yaw",
    (4, 2) : "left_elbow_pitch",
    (3, 2) : "left_shoulder_twirl",
    (2, 2) : "left_shoulder_roll",
    (1, 2) : "left_shoulder_pitch",
    (0, 2) : "hip",
    (10,1) : "right_ankle_roll",
    (9,1) : "right_ankle_pitch",
    (8,1) : "right_knee",
    (7,1) : "right_hip_pitch",
    (6,1) : "right_hip_roll",
    (5,1) : "right_hip_yaw",
    (4,1) : "right_elbow_pitch",
    (3,1) : "right_shoulder_twirl",
    (2,1) : "right_shoulder_roll",
    (1,1) : "right_shoulder_pitch",
    (0, 1) : "head"
  
}

class ServoData:
        def __init__(self, id=0, sio=0, data=0):
            self.id = id
            self.sio = sio
            self.data = data
            
            self.ind = (id, sio)
            self.name = servos[self.ind]

        def __lt__(self, other):
            return (self.id * 2 + (self.sio - 1)) < (other.id * 2 + (other.sio - 1))
        
        def __repr__(self):
            return "({0}, {1}, {2})".format(self.id, self.sio, self.data)

class Kondocontr():
    def __init__(self, robot):
        self.robot = robot
        #инициализация иму сенсоров
        self.robot.getDevice("imu_head").enable(1)
        self.robot.getDevice("imu_body").enable(1)

    def setServoPos (self, servoDatas, frame):
        for f in range(1):
            for sD in servoDatas:
                #print(sD.name)
                if str(sD.name) == "hip" or str(sD.name) ==  "head":
                    continue
                self.robot.getDevice(str(sD.name)).setPosition(sD.data) 

                #print(sD.data)
        #time.sleep(0.001)
    def sendServos (self, servoDatas, frame):
        servos = servos_msg2_pb2_pb2.ActuatorRequests()
        for sD in servoDatas:
                serv = servos.motor_positions.add() 
                serv.name = str(sD.name)
                serv.position = sD.data
        f = open("msg", "wb")
        f.write(servos.SerializeToString())
        f.close()   

    #функция получения данных roll pitch yaw с сенсора с именем name    
    def getData(self, name):
        return self.robot.getDevice(name).getRollPitchYaw()

    #функции получения координат мяча и робота
    def getPositionBall(self):
        return self.robot.getFromDef("ball").getPosition()

    def getPositionRobot(self):
        return self.robot.getSelf().getPosition()



    def walk_Initial_Pose(self):
        self.xtr = self.xtl = 0
        framestep = self.simThreadCycleInMs//10
        ii = 0
        for j in range (self.initPoses):
            start1 = time.perf_counter()
            self.ztr = -223.0 + j*(223.0-self.gaitHeight)/self.initPoses
            self.ztl = -223.0 + j*(223.0-self.gaitHeight)/self.initPoses
            self.ytr = -self.d10 - j*self.amplitude/2 /self.initPoses
            self.ytl =  self.d10 - j*self.amplitude/2 /self.initPoses
            angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
            #if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                servoDatas = []
                for i in range(len(angles)):
                    #pos = int(angles[i]*1698 + 7500)
                    pos = angles[i]
                    servoDatas.append(ServoData(self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos))
                servoDatas = self.reOrderServoData(servoDatas)
                a=self.kondo.setServoPos (servoDatas, self.frames_per_cycle)
                time1 = time.perf_counter() - start1
                robot.step(timestep)
                

    
if __name__=="__main__":
    kondo = Kondocontr(robot)
    while robot.step(32) != -1:
        print("Hello World!")
        position = kondo.getPositionRobot()
        pos = servos_msg2_pb2_pb2.Position()
        #posi = pos.Position.add() 
        
        pos.X = position[0]
        pos.Y = position[1]
        data = pos.SerializeToString()
        socket_pos.send(data)
