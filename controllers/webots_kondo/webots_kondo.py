from controller import *
import sys

maxServAngle = 4.712

#для получения информации о мире robot->supervisor
robot = Supervisor()

timestep = int(robot.getBasicTimeStep())


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
            
            

        def icsNum2id(self):
            return self.id * 2 + (self.sio - 1)

        def itemAdd(self, x, y, z):
            self.id = x
            self.sio = y
            self.data = z

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

    #функция получения данных roll pitch yaw с сенсора с именем name    
    def getData(self, name):
        return self.robot.getDevice(name).getRollPitchYaw()

    #функции получения координат мяча и робота
    def getPositionBall(self):
        return self.robot.getFromDef("ball").getPosition()

    def getPositionRobot(self):
        return self.robot.getSelf().getPosition()

#  Walking engine for Starkit Kondo OpenMV
#  Copyright STARKIT Soccer team of MIPT

import sys
import math, time
from compute_Alpha_v3 import Alpha

class Motion:
    def __init__(self):
        self.ACTIVESERVOS = [(10,2),(9,2),(8,2),(7,2),(6,2),(5,2),(4,2),
                (3,2),(2,2),(1,2),(0,2),(10,1),(9,1),(8,1),
                (7,1),(6,1),(5,1),(4,1),(3,1),(2,1),(1,1)]
        # (10,2) Прав Стопа Вбок, (9,2) Прав Стопа Вперед,(8,2) Прав Голень, (7,2) Прав Колено,
        # (6,2)  Прав Бедро,      (5,2) Прав Таз,         (1,2) Прав Ключица,(4,2) Прав Локоть, (0,2) Торс
        # (10,1) Лев Стопа Вбок,  (9,1) Лев Стопа Вперед, (8,1) Лев Голень,  (7,1) Лев Колено
        # (6,1)  Лев Бедро,       (5,1) Лев Таз,          (1,1) Лев Ключица, (4,1) Лев Локоть

        a5 = 21.5  # мм расстояние от оси симметрии до оси сервы 5
        b5 = 18.5  # мм расстояние от оси сервы 5 до оси сервы 6 по горизонтали
        c5 = 0     # мм расстояние от оси сервы 6 до нуля Z по вертикали
        a6 = 42    # мм расстояние от оси сервы 6 до оси сервы 7
        a7 = 65.5  # мм расстояние от оси сервы 7 до оси сервы 8
        a8 = 63.8  # мм расстояние от оси сервы 8 до оси сервы 9
        a9 = 35.5  # мм расстояние от оси сервы 9 до оси сервы 10
        a10= 25.4  # мм расстояние от оси сервы 10 до центра стопы по горизонтали
        b10= 16.4  # мм расстояние от оси сервы 10 до низа стопы
        c10 = 12   # мм расстояние от оси сервы 6 до оси сервы 10 по горизонтали
        self.SIZES = [ a5, b5, c5, a6, a7, a8, a9, a10, b10, c10 ]
        self.d10 = 53.4 #53.4 # расстояние по Y от центра стопы до оси робота
        limAlpha5 = [-2667, 2667]
        limAlpha6 = [-3000,  740]
        limAlpha7 = [-3555, 3260]
        limAlpha8 = [-4150, 1777]
        limAlpha9 = [-4000, 2960]
        limAlpha10 =[-2815,   600]
        LIMALPHA = [limAlpha5, limAlpha6, limAlpha7, limAlpha8, limAlpha9, limAlpha10]
        self.MOTION_SLOT_DICT = {0:['',0], 1:['',0], 2:['',0], 3:['',0], 4:['',0], 5:['Get_Up_Inentification',7000],
                    6:['Soccer_Get_UP_Stomach_N', 5000], 7:['Soccer_Get_UP_Face_Up_N', 5000], 8:['Soccer_Walk_FF',0], 9:['',0], 10:['',0],
                    11:['',0], 12:['',0], 13:['',0], 14:['Soccer_Small_Jump_Forward',0], 15:['',0],
                    16:['',0], 17:['',0], 18:['Soccer_Kick_Forward_Right_Leg',5000], 19: ['Soccer_Kick_Forward_Left_Leg',5000], 20:['',0],
                    21:['Get_Up_From_Defence',1000], 22:['',0], 23:['PanaltyDefenceReady_Fast',500], 24:['PenaltyDefenceF',300], 25:['',0],
                    26:['',0], 27:['',0], 28:['',0], 29:['',0], 30:['Soccer_Walk_FF0',0],
                    31:['Soccer_Walk_FF1',0], 32:['Soccer_Walk_FF2',0], 33: ['Soccer_Get_UP_Stomach',0], 34:['Soccer_Get_UP_Face_Up',0],
                    35: ['Get_Up_Right',0], 36: ['PenaltyDefenceR',2000], 37: ['PenaltyDefenceL',2000]}
        self.TIK2RAD = 0.00058909
        self.frame_delay = 25
        self.frames_per_cycle = 2
        self.stepLength = 0.0    # -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
        self.sideLength = 0.0         # -20 - +20. Side step length to right (+) and to left (-)
        self.rotation = 0           # -45 - +45 degrees Centigrade per step + CW, - CCW.
        self.first_Leg_Is_Right_Leg = True
        # Following paramenetrs Not recommended for change
        self.amplitude = 32          # mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2
        self.fr1 =8                  # frame number for 1-st phase of gait ( two legs on floor)
        self.fr2 = 12                # frame number for 2-nd phase of gait ( one leg in air)
        self.gaitHeight= 180         # Distance between Center of mass and floor in walk pose
        self.stepHeight = 32.0       # elevation of sole over floor
        self.simThreadCycleInMs = 20
        self.initPoses = 400//self.simThreadCycleInMs
        self.limAlpha1 =LIMALPHA
        self.limAlpha1[3][1]=0
        #  end of  paramenetrs Not recommended for change
        self.al = Alpha()
        self.exitFlag = 0
        self.falling_Flag = 0
        self.activePose = []
        self.xtr = 0
        self.ytr = -self.d10   #-53.4
        self.ztr = -self.gaitHeight
        self.xr = 0
        self.yr = 0
        self.zr = -1
        self.wr = 0
        self.xtl = 0
        self.ytl = self.d10   # 53.4
        self.ztl = -self.gaitHeight
        self.xl = 0
        self.yl = 0
        self.zl = -1
        self.wl = 0
        self.ACTIVEJOINTS = ['Leg_right_10','Leg_right_9','Leg_right_8','Leg_right_7','Leg_right_6','Leg_right_5','hand_right_4',
            'hand_right_3','hand_right_2','hand_right_1','Tors1','Leg_left_10','Leg_left_9','Leg_left_8',
            'Leg_left_7','Leg_left_6','Leg_left_5','hand_left_4','hand_left_3','hand_left_2','hand_left_1','head0','head12']

        self.kondo = Kondocontr(robot)
       
        


    #-------------------------------------------------------------------------------------------------------------------------------

    def falling_Test(self):
        ad3 = self.kondo.getAdData(3)
        ad4 = self.kondo.getAdData(4)
        if ad3 < 200:
            self.falling_Flag = 1     # on stomach
            self.kondo.motionPlay(6)
            self.activePose = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            time.sleep(6)
        if ad3 > 450:
            self.falling_Flag = -1    # face up
            self.kondo.motionPlay(7)
            self.activePose = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            time.sleep(6)
        if ad4 > 400:
            self.falling_Flag = -2    # on right side
            self.kondo.motionPlay(5)
            self.activePose = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            time.sleep(8)
        if ad4 < 160:
            self.falling_Flag = 2     # on left side
            self.kondo.motionPlay(5)
            self.activePose = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            time.sleep(8)
        return 0

    def computeAlphaForWalk(self,sizes, limAlpha, hands_on = True ):
        angles =[]
        anglesR=[]
        anglesL=[]
        anglesR = self.al.compute_Alpha_v3(self.xtr,self.ytr,self.ztr,self.xr,self.yr,self.zr,self.wr, sizes, limAlpha)
        anglesL = self.al.compute_Alpha_v3(self.xtl,-self.ytl,self.ztl,self.xl,-self.yl,self.zl,self.wl, sizes, limAlpha)
        if len(anglesR)>1:
            for i in range(len(anglesR)):
                if len(anglesR)==1: break
                if anglesR[0][2]<anglesR[1][2]: anglesR.pop(1)
                else: anglesR.pop(0)
        elif len(anglesR)==0:
            return[]
        if len(anglesL)>1:
            for i in range(len(anglesL)):
                if len(anglesL)==1: break
                if anglesL[0][2]<anglesL[1][2]: anglesL.pop(1)
                else: anglesL.pop(0)
        elif len(anglesL)==0:
            return[]
        if self.first_Leg_Is_Right_Leg == True:
            for j in range(6): angles.append(anglesR[0][j])
            if hands_on: angles.append(1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(0.524 - self.xtl/57.3)
            else: angles.append(0.0)
            angles.append(0.0)
            #for j in range(5): angles.append(0.0)
            for j in range(6): angles.append(-anglesL[0][j])
            #for j in range(4): angles.append(0.0)
            if hands_on: angles.append(-1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(-0.524 + self.xtr/57.3)
            else: angles.append(0.0)
        else:
            for j in range(6): angles.append(anglesL[0][j])
            if hands_on: angles.append(1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(0.524 - self.xtr/57.3)
            else: angles.append(0.0)
            angles.append(0.0)                                  # Tors
            for j in range(6): angles.append(-anglesR[0][j])
            if hands_on: angles.append(-1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(-0.524 + self.xtl/57.3)
            else: angles.append(0.0)
        self.activePose = angles
        return angles

    def reOrderServoData(self, servoDatas):
        order = [0, 11, 1, 12, 2, 13, 3, 14, 4, 15, 5, 16, 6, 17, 7, 18, 8, 19, 9, 20, 10]
        servoDatasOrdered = []
        for orderNumber in order:
            servoDatasOrdered.append(servoDatas[orderNumber])
        return servoDatasOrdered

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
                

    def walk_Cycle(self, stepLength,sideLength, rotation,cycle, number_Of_Cycles):
        #получение углов
        print("head imu", self.kondo.getData("imu_head"))
        print("body imu", self.kondo.getData("imu_head"))
        self.stepLength = stepLength
        self.sideLength = sideLength
        self.rotation = math.degrees(rotation)
        rotation = -self.rotation/222
        alpha = 0
        alpha01 = math.pi/self.fr1*2
        frameNumberPerCycle = 2*self.fr1+2*self.fr2
        framestep = self.simThreadCycleInMs//10
        xtl0 = self.stepLength * (1 - (self.fr1 + self.fr2 + 2 * framestep) / (2*self.fr1+self.fr2+ 2 * framestep)) * 1.5     # 1.5 - podgon
        xtr0 = self.stepLength * (1/2 - (self.fr1 + self.fr2 + 2 * framestep ) / (2*self.fr1+self.fr2+ 2 * framestep))
        dx0_typical = self.stepLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion forward per framestep
        dy0_typical = self.sideLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion sideways per framestep
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of body tilt forward
        self.xr, self.xl = 0.01, 0.01 # ['BODY_TILT_AT_WALK']
        for iii in range(0,frameNumberPerCycle,framestep):
            

            start1 = time.perf_counter()
            if 0<= iii <self.fr1 :                                              # FASA 1
                alpha = alpha01 * (iii/2+0.5*framestep)
                #alpha = alpha01 * iii/2
                S = (self.amplitude/2 + self.sideLength/2 )*math.cos(alpha)
                self.ytr = S - self.d10 + self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                if cycle ==0: continue
                else: dx0 = dx0_typical
                self.xtl = xtl0 - dx0 - dx0 * iii/framestep
                self.xtr = xtr0 - dx0 - dx0 * iii/framestep

            if self.fr1+self.fr2<=iii<2*self.fr1+self.fr2 :                     # FASA 3
                alpha = alpha01 * ((iii-self.fr2)/2+0.5*framestep)
                #alpha = alpha01 * (iii-self.fr2)/2
                S = (self.amplitude/2 + self.sideLength/2)*math.cos(alpha)
                self.ytr = S - self.d10 - self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                dx0 = dx0_typical
                self.xtl -= dx0
                self.xtr -= dx0

            if self.fr1<= iii <self.fr1+self.fr2:                               # FASA 2
                self.ztr = -self.gaitHeight + self.stepHeight
                if cycle ==0:
                    dx = self.stepLength/(self.fr2- 2 * framestep)*framestep/2
                    dx0 = dx0_typical
                    dy = self.sideLength/self.fr2*framestep
                    dy0 = dy0_typical
                else:
                    dx = self.stepLength/(self.fr2- 2 * framestep)*framestep #* 0.75
                    dx0 = dx0_typical
                    dy = self.sideLength/self.fr2*framestep
                    dy0 = dy0_typical
                if iii==self.fr1:
                    self.xtr -= dx0
                    #self.ytr = S - 64 + dy0
                    self.ytr = S - self.d10 + dy0
                elif iii == (self.fr1 +self.fr2 - framestep):
                    self.xtr -= dx0
                    self.ytr = S - self.d10 + 2*dy0 - self.sideLength
                else:
                    self.xtr += dx
                    self.ytr = S - 64 + dy0 - dy*self.fr2/(self.fr2- 2 * framestep)*((iii - self.fr1)/2)
                    self.wr = self.wl = rotation -(iii-self.fr1)* rotation/(self.fr2- 2 * framestep)*2
                self.xtl -= dx0
                self.ytl += dy0

            if 2*self.fr1+self.fr2<= iii :                                         # FASA 4
                self.ztl = -self.gaitHeight + self.stepHeight
                if cycle == number_Of_Cycles - 1:
                    dx0 = dx0_typical * 4 / self.fr2           # 8.75/6
                    dx = (self.stepLength*(self.fr1+self.fr2)/(4*self.fr1)+2*dx0)/(self.fr2- 2 * framestep) *framestep / 1.23076941   # 1.23076941 = podgon
                    if iii== (2*self.fr1 + 2*self.fr2 - framestep):
                        self.ztl = -self.gaitHeight
                        self.ytl = S + self.d10
                else:
                    dx = self.stepLength/(self.fr2- 2 * framestep) *framestep # * 0.75
                    dx0 = dx0_typical
                    dy = self.sideLength/(self.fr2- 2 * framestep) *framestep
                    dy0 = dy0_typical
                if iii== (2*self.fr1 + self.fr2 ):
                    self.xtl -= dx0
                    #self.ytl = S + 64 + dy0
                    self.ytl = S + self.d10 + dy0
                elif iii== (2*self.fr1 + 2*self.fr2 - framestep):
                    self.xtl -= dx0
                    self.ytl = S + self.d10 + 2*dy0 - self.sideLength
                else:
                    self.xtl += dx
                    self.ytl = S + 64 + dy0 - dy * (iii -(2*self.fr1+self.fr2) )/2
                    self.wr = self.wl = (iii-(2*self.fr1+self.fr2))* rotation/(self.fr2- 2 * framestep) *2 - rotation
                self.xtr -= dx0
                self.ytr += dy0
            angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
            #print('iii = ', iii, 'ytr =', self.ytr, 'ytl =', self.ytl)
            
            servoDatas = []
            disp = []
            for i in range(len(angles)):
                #pos = int(angles[i]*1698 + 7500)
                pos = angles[i]
                disp.append((self.ACTIVEJOINTS[i],self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos -7500))
                servoDatas.append(ServoData(self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos))
            servoDatas = self.reOrderServoData(servoDatas)
            a=self.kondo.setServoPos (servoDatas, self.frames_per_cycle)
            robot.step(timestep)
            
            #time.sleep(0.001)
            #print('disp[4] = ', disp[4], 'disp[15]=', disp[15])
            time1 = time.perf_counter() - start1
        # returning xr, xl, yr, yl to initial value
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old

    def walk_Final_Pose(self):

        framestep = self.simThreadCycleInMs//10
        for j in range (self.initPoses):
            start1 = time.perf_counter()
            self.ztr = -self.gaitHeight - (j+1)*(223.0-self.gaitHeight)/self.initPoses
            self.ztl = -self.gaitHeight - (j+1)*(223.0-self.gaitHeight)/self.initPoses
            self.ytr = -self.d10 - (self.initPoses-(j+1))*self.amplitude/2 /self.initPoses
            self.ytl =  self.d10 - (self.initPoses-(j+1))*self.amplitude/2 /self.initPoses
            if j == self.initPoses - 1:
                angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1, hands_on = False)
            else: angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
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
                #print(servoDatas)
                #print(clock.avg())
                time1 = time.perf_counter() - start1
               



if __name__=="__main__":
    motion = Motion()
    number_Of_Cycles = 0
    stepLength = 64
    sideLength = 0
    rotation = 0
    motion.walk_Initial_Pose()
    robot.step(timestep)

    number_Of_Cycles += 1
    for cycle in range(number_Of_Cycles):    
        robot.step(timestep)      
        stepLength1 = stepLength
        if cycle ==0 : stepLength1 = stepLength/3
        if cycle ==1 : stepLength1 = stepLength/3 * 2
        motion.walk_Cycle(stepLength1,sideLength, rotation,cycle, number_Of_Cycles)
        motion.walk_Final_Pose()
    ###
    while(True):
        #пример вызова получения координат
        print("position of robot", motion.kondo.getPositionRobot())
        print("position of ball", motion.kondo.getPositionBall())
        #пример вызова иму 
        #print("head imu", motion.kondo.getData("imu_head"))
        #print("body imu", motion.kondo.getData("imu_head"))
        robot.step(timestep)
    



