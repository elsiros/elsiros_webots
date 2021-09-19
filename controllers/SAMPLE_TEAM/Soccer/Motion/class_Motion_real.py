"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of A. Babaev.

The module is a part of motion generating functions
"""
import math, time, json
import logging

from .class_Motion import Motion1
from .ball_Approach_Steps_Seq import *
from .path_planning import PathPlan

class Motion_real(Motion1):

    def __init__(self, glob):
        super().__init__(glob)
        self.p = PathPlan(self.glob)

    def seek_Ball_In_Pose(self, fast_Reaction_On, penalty_Goalkeeper = False, with_Localization = True):
        self.local.correct_yaw_in_pf()
        if self.robot_In_0_Pose == False:
            self.simulateMotion(name = 'Initial_Pose')
            self.robot_In_0_Pose = True
        variants = []
        # U19 - Шея поворот
        # U20 - Шея Наклон
        c = self.neck_play_pose
        head_pose = [(-2667,c), (-1333, c) , ( 0, c) , (1333, c) , (2667,c),
                     (-2667, c-700),(-1333, c-700), (0, c-700), (1333,c-700),(2667, c-700),
                    (-2667, c-1400), (-1333, c-1400), ( 0, c-1400), (1333, c-1400), (2667, c-1400)]
        #head_pose_seq = [2,7,12,13,8,3,1,6,11,10,5,0,4,9,14,2]
        head_pose_seq = [2,7,12,11,6,8,13,14,9,4,3,10,5,0,1,2]
        if penalty_Goalkeeper: head_pose_seq = [2,7,12,11,6,8,13]
        for i in range(len(head_pose_seq)):
            if not(fast_Reaction_On == True and i == 0):
                x = head_pose[head_pose_seq[i]]
                self.neck_pan = x[0]
                self.neck_tilt = x[1]
            if not self.falling_Test() == 0:
                self.local.quality =0
                if self.falling_Flag == 3: self.logger.debug('STOP!')
                else: self.logger.debug('FALLING!!!' + str(self.falling_Flag))
                return False, 0, 0, [0, 0]
            self.move_head(self.neck_pan, self.neck_tilt)
            self.refresh_Orientation()
            a, course, dist = self.seek_Ball_In_Frame(with_Localization)
            if a == True: 
                variants.append ((course, dist *1000))
            if fast_Reaction_On == True and a== True: break
        course = 0
        distance = 0
        if len(variants)>0:
            for i in range (len(variants)):
                course = course + variants[i][0]
                distance = distance + variants[i][1]
            course1  = course /len(variants)
            distance1 = distance /len(variants)
            self.neck_pan =int( - course1/ self.TIK2RAD)
            D = self.params['HEIGHT_OF_CAMERA'] - self.params['HEIGHT_OF_NECK']- self.params['DIAMETER_OF_BALL']/2
            E = (2*distance1*D - math.sqrt(4*distance1**2*D**2 - 4*(distance1**2-self.params['HEIGHT_OF_NECK']**2)*(D**2 -self.params['HEIGHT_OF_NECK']**2)))/(2*(D**2-self.params['HEIGHT_OF_NECK']**2))
            alpha = math.atan(E)
            alpha_d = math.pi/2 - alpha
            self.neck_tilt = int((-alpha_d)/self.TIK2RAD + self.neck_calibr)
            #self.logger.debug('self.neck_pan =' + str(self.neck_pan) + 'self.neck_tilt =' + str(self.neck_tilt))
            self.move_head(self.neck_pan, self.neck_tilt)
            self.refresh_Orientation()
            a, course, dist, speed = self.detect_Ball_Speed(with_Localization)
            if with_Localization: self.local.localisation_Complete()
            #self.local.pf_update()
            if a == True:
                #course_global = course + self.euler_angle[0] + self.neck_pan * 0.03375
                if with_Localization: self.local.localisation_Complete()
                course_global_rad = course + self.glob.pf_coord[2]
                self.glob.ball_coord = [dist*math.cos(course_global_rad)+ self.glob.pf_coord[0],
                                         dist*math.sin(course_global_rad)+ self.glob.pf_coord[1]]
                #if len(self.glob.obstacles) == 0: self.glob.obstacles = [[0,0,0]]
                #self.glob.obstacles[0] = [self.glob.ball_coord[0], self.glob.ball_coord[1], 0.15]
                if with_Localization: self.local.localisation_Complete()
                #if self.glob.obstacleAvoidanceIsOn: self.sim_Get_Obstacles()
                return(a, course, dist, speed)
            else:
                if distance1 !=0:
                    if with_Localization: self.local.localisation_Complete()
                    dist = distance1 / 1000
                    course_global_rad = course1 + self.glob.pf_coord[2]
                    self.glob.ball_coord = [dist * math.cos(course_global_rad) + self.glob.pf_coord[0],
                                            dist * math.sin(course_global_rad) + self.glob.pf_coord[1]]
                    return(a, course1, dist, [0, 0])
        if with_Localization: self.local.localisation_Complete()
        return False, 0, 0, [0, 0]

    def watch_Ball_In_Pose(self, penalty_Goalkeeper = False):
        self.local.correct_yaw_in_pf()
        if self.robot_In_0_Pose == False:
            self.simulateMotion(name = 'Initial_Pose')
            self.robot_In_0_Pose = True
        # U19 - Шея поворот
        # U20 - Шея Наклон
        c = self.neck_play_pose
        head_pose = [(-2667,c), (-1333, c) , ( 0, c) , (1333, c) , (2667,c),
                     (-2667, c-700),(-1333, c-700), (0, c-700), (1333,c-700),(2667, c-700),
                    (-2667, c-1400), (-1333, c-1400), ( 0, c-1400), (1333, c-1400), (2667, c-1400)]
        #head_pose_seq = [2,7,12,13,8,3,1,6,11,10,5,0,4,9,14,2]
        head_pose_seq = [2,7,12,11,6,8,13,14,9,4,3,10,5,0,1,2]
        if penalty_Goalkeeper: head_pose_seq = [2,7,12,11,6,8,13]
        for i in range(len(head_pose_seq)):
            if i != 0:
                x = head_pose[head_pose_seq[i]]
                self.neck_pan = x[0]
                self.neck_tilt = x[1]
            if not self.falling_Test() == 0:
                self.local.quality =0
                if self.falling_Flag == 3: self.logger.debug('STOP!')
                else: self.logger.debug('FALLING!!!' + str(self.falling_Flag))
                return False, 0, 0, [0, 0]
            self.move_head(self.neck_pan, self.neck_tilt)
            self.refresh_Orientation()
            a, course, dist, speed = self.detect_Ball_Speed(with_Localization = False)
            if a == True or (a== False and dist !=0): break
        if a == True or (a== False and dist !=0):
            course_global_rad = course + self.glob.pf_coord[2]
            self.glob.ball_coord = [dist*math.cos(course_global_rad)+ self.glob.pf_coord[0],
                                        dist*math.sin(course_global_rad)+ self.glob.pf_coord[1]]
            #if len(self.glob.obstacles) == 0: self.glob.obstacles = [[0,0,0]]
            #self.glob.obstacles[0] = [self.glob.ball_coord[0], self.glob.ball_coord[1], 0.15]
            #if self.glob.obstacleAvoidanceIsOn: self.sim_Get_Obstacles()
            distance = dist *1000
            self.neck_pan =int( - course/ self.TIK2RAD)
            D = self.params['HEIGHT_OF_CAMERA'] - self.params['HEIGHT_OF_NECK']- self.params['DIAMETER_OF_BALL']/2
            E = (2*distance*D - math.sqrt(4*distance**2*D**2 - 4*(distance**2-self.params['HEIGHT_OF_NECK']**2)*(D**2 -self.params['HEIGHT_OF_NECK']**2)))/(2*(D**2-self.params['HEIGHT_OF_NECK']**2))
            alpha = math.atan(E)
            alpha_d = math.pi/2 - alpha
            self.neck_tilt = int((-alpha_d)/self.TIK2RAD + self.neck_calibr)
            return(a, course, dist, speed)
        return False, 0, 0, [0, 0]

    def seek_Ball_In_Frame(self, with_Localization = True):
        #self.pause_in_ms(100)
        Ballposition = self.sim_Get_Ball_Position()
        if with_Localization: self.local.read_Localization_marks()
        self.logger.debug('Ballposition: ' + str(Ballposition))
        if Ballposition:
            course, distance = Ballposition
            return True, course, distance
        else: return False, 0, 0

    def detect_Ball_Speed(self, with_Localization = False):
        position = []
        if with_Localization : self.local.read_Localization_marks()
        for number in range (2):
            #self.pause_in_ms(100)
            Ballposition = self.sim_Get_Ball_Position()
            if Ballposition:
                course, distance = Ballposition
                position.append([course,distance])
        n = len(position)
        speed = [0,0]
        if n > 1:
            front_speed = ( position[n-1][1] - position[0][1])/ distance/n
            tangential_speed = ( position[n-1][0] - position[0][0]) * distance/n
            speed = [tangential_speed, front_speed ]
        if n < 1: return False, 0, 0, [0,0]
        
        elif n < 2: 
            #self.see_ball_confirmation()
            return False, course, distance, [0,0]
        else: 
            #self.see_ball_confirmation()
            return True, course, distance, speed

    def see_ball_confirmation(self):
        self.move_head(self.neck_pan, 0)
        self.move_head(self.neck_pan, self.neck_tilt)

    def turn_To_Course(self, course, accurate = False):
        stepLength = 0
        sideLength = 0
        rotation = 0
        cycleNumber = 1
        cycle = 0
        old_neck_pan, old_neck_tilt = self.head_Up()
        self.refresh_Orientation()
        rotation1 = course - self.imu_body_yaw()
        rotation1 = self.norm_yaw(rotation1)
        if abs(rotation1)> 0.035 or accurate:
            cycleNumber = int(math.floor(abs(rotation1)/self.params['ROTATION_YIELD']))+1       # rotation yield 0.23 with rotation order 0.21
            self.walk_Initial_Pose()
            for cycle in range (cycleNumber):
                rotation1 = course - self.imu_body_yaw()
                rotation1 = self.norm_yaw(rotation1)
                if abs(rotation1)< 0.035 and not accurate: break
                if abs(rotation1)< 0.01: break
                rotation = rotation1/(cycleNumber - cycle)
                self.walk_Cycle(stepLength, sideLength,rotation,cycle,cycleNumber)
            self.walk_Final_Pose()
        self.refresh_Orientation()
        self.local.coord_shift = [0,0,0]
        self.local.coordinate_record()
        self.head_Return(old_neck_pan, old_neck_tilt)

    def head_Up(self):
        old_neck_pan = self.neck_pan
        old_neck_tilt = self.neck_tilt
        self.neck_pan = 0
        self.neck_tilt = self.neck_play_pose
        self.move_head(self.neck_pan, self.neck_tilt)
        self.refresh_Orientation()
        return old_neck_pan, old_neck_tilt

    def head_Return(self, old_neck_pan, old_neck_tilt):
        self.move_head(old_neck_pan, old_neck_tilt)
        self.refresh_Orientation()

    def localisation_Motion(self):
        if not self.falling_Test() == 0:
            self.local.quality =0
            if self.falling_Flag == 3: self.logger.debug('STOP!')
            else: self.logger.debug('FALLING!!!' + str(self.falling_Flag))
            return[]
        if self.robot_In_0_Pose == False:
            self.simulateMotion(name = 'Initial_Pose')
            self.robot_In_0_Pose = True
        # U19 - Шея поворот
        # U20 - Шея Наклон
        c = self.neck_play_pose
        head_pose = [(-2667,c), (-1333, c) , ( 0, c) , (1333, c) , (2667,c),
                     (-2667, c-700),(-1333, c-700), (0, c-700), (1333,c-700),(2667, c-700),
                    (-2667, c-1400), (-1333, c-1400), ( 0, c-1400), (1333, c-1400), (2667, c-1400)]
        #head_pose_seq = [2,7,12,11,6,8,13,14,9,4,3,10,5,0,1,2]
        head_pose_seq = [2,7,6,8,9,4,3,5,0,1,2]
        for k in range(1):
            for i in range(len(head_pose_seq)):
                x = head_pose[head_pose_seq[i]]
                self.neck_pan = x[0]
                self.neck_tilt = x[1]
                self.move_head(self.neck_pan, self.neck_tilt)
                self.refresh_Orientation()
                a, course, distance, blob = self.seek_Ball_In_Frame()
            #self.local.pf_update()
            #a = self.local.process_Post_data_in_Pose()
            #if self.local.quality == 1 : break
            #target_course1 = self.euler_angle[0] +180
            #self.turn_To_Course(target_course1)
        a = self.local.localisation_Complete()
        #self.local.pf_update()
        return a

    def normalize_rotation(self, yaw):
        if abs(yaw) > 2 * math.pi: yaw %= (2 * math.pi)
        if yaw > math.pi : yaw -= (2 * math.pi)
        if yaw < -math.pi : yaw += (2 * math.pi)
        if yaw > 0.5 : yaw = 0.5
        if yaw < -0.5 : yaw = -0.5
        return yaw

    def near_distance_omni_motion(self, dist_mm, napravl):
        old_neck_pan, old_neck_tilt = self.head_Up()
        dist = dist_mm/1000
        #self.refresh_Orientation()
        initial_direction = self.imu_body_yaw()
        self.logger.debug('initial_direction' + str(initial_direction))
        n = int(math.floor((dist_mm*math.cos(napravl)-self.first_step_yield)/self.cycle_step_yield)+1)+1         #calculating the number of potential full steps forward
        displacement = dist_mm*math.sin(napravl)
        if displacement > 0:
            invert = -1
            self.first_Leg_Is_Right_Leg = False
            side_step_yield = self.side_step_left_yield
        else:
            invert = 1
            side_step_yield = self.side_step_right_yield
        m = int(math.floor(abs(displacement)/side_step_yield)+1)
        if n < m : n = m
        stepLength = dist_mm*math.cos(napravl)/(self.first_step_yield*1.25+self.cycle_step_yield*(n-1)+ self.cycle_step_yield*0.75)*64
        number_Of_Cycles = n+2
        sideLength = abs(displacement) /number_Of_Cycles*20/side_step_yield
        if stepLength > 15 and number_Of_Cycles > 4: 
            deceleration = True
            number_Of_Cycles += 1
        else: deceleration = False
        #old_neck_pan, old_neck_tilt = self.head_Up()
        self.local.correct_yaw_in_pf()
        self.walk_Initial_Pose()
        for cycle in range(number_Of_Cycles):
            #self.refresh_Orientation()
            rotation = initial_direction - self.imu_body_yaw() * 1
            rotation = self.normalize_rotation(rotation)
            stepLength1 = stepLength
            if cycle == 0: stepLength1 = stepLength/4
            if cycle == 1: stepLength1 = stepLength/2
            if deceleration:
                if cycle == number_Of_Cycles - 1: stepLength1 = stepLength / 3
                if cycle == number_Of_Cycles - 2: stepLength1 = stepLength * 2 / 3
            self.walk_Cycle(stepLength1, sideLength, invert*rotation,cycle,number_Of_Cycles)
        self.walk_Final_Pose()
        self.first_Leg_Is_Right_Leg = True
        self.head_Return(old_neck_pan, old_neck_tilt)

    def near_distance_ball_approach_and_kick(self, kick_direction, strong_kick = False, small_kick = False ):
        offset_of_ball = self.params['KICK_OFFSET_OF_BALL']  # self.d10 # module of local robot Y coordinate of ball im mm before kick 
        a, napravl, dist, speed = self.seek_Ball_In_Pose(fast_Reaction_On = True)
        dist_mm = dist *1000
        if a==False or self.falling_Flag != 0: return False
        if dist > 0.9 or a == False: return False
        if  0.02 < abs(dist * math.cos(napravl)) < 0.06 and dist * math.sin(napravl) < 0.03:
            old_neck_pan, old_neck_tilt = self.head_Up()
            if napravl > 0: self.kick(first_Leg_Is_Right_Leg=False)
            else: self.kick(first_Leg_Is_Right_Leg=True)
            self.head_Return(old_neck_pan, old_neck_tilt)
        if abs(napravl) > 1 :
            direction = math.copysign(2.55, napravl)
            self.near_distance_omni_motion( 180 , direction)
        else:
            forth_dist = dist_mm*math.cos(napravl) 
            n = int(math.ceil((forth_dist - self.params['KICK_ADJUSTMENT_DISTANCE']
                                -self.first_step_yield)/self.cycle_step_yield)+1)         #calculating the number of potential full steps forward
            displacement = dist_mm*math.sin(napravl)- math.copysign(offset_of_ball, napravl)
            if displacement > 0:
                invert = -1
                self.first_Leg_Is_Right_Leg = False
                side_step_yield = self.side_step_left_yield
            else:
                invert = 1
                side_step_yield = self.side_step_right_yield
            m = int(math.ceil(abs(displacement)/side_step_yield))       # potential side steps number
            if n < m : n = m
            n += 2                                                      # final steps number
            stepLength = (dist_mm*math.cos(napravl)-
                          self.params['KICK_ADJUSTMENT_DISTANCE'])/(self.first_step_yield
                          + self.cycle_step_yield * n) * 64
            number_Of_Cycles = n + 1
            if napravl > 0:
                kick_by_Right = False
            else:
                kick_by_Right = True
            sideLength = abs(displacement)/number_Of_Cycles*20/side_step_yield
            old_neck_pan, old_neck_tilt = self.head_Up()
            self.local.correct_yaw_in_pf()
            init_yaw = kick_direction = self.imu_body_yaw()
            #init_yaw = self.imu_body_yaw()
            stepLengthResidue = 0
            sideLengthResidue = 0
            self.walk_Initial_Pose()
            cycle = 0
            while (cycle < number_Of_Cycles):
            #for cycle in range(number_Of_Cycles):
                #self.refresh_Orientation()
                rotation = (kick_direction - self.imu_body_yaw()) * 1
                rotation = self.normalize_rotation(rotation)
                stepLength1 = stepLength
                sideLength1 = sideLength
                self.logger.debug('kick_direction =' + str(kick_direction) + ' self.imu_body_yaw() = ' + str(self.imu_body_yaw()) + ' rotation = ' + str(rotation) )
                if cycle == 0: stepLength1 = stepLength / 3
                if cycle == 1: stepLength1 = stepLength * 2 / 3
                stepLength1 += stepLengthResidue
                sideLength1 += sideLengthResidue
                self.walk_Cycle(stepLength1, sideLength1, invert*rotation,cycle,number_Of_Cycles)
                delta_yaw = self.norm_yaw(self.imu_body_yaw() - init_yaw)
                stepLengthResidue = stepLength1 * (1 - math.cos(delta_yaw)) - sideLength1 * math.sin(delta_yaw) * invert
                sideLengthResidue = sideLength1 * (1 - math.cos(delta_yaw)) + stepLength1 * math.sin(delta_yaw) * invert
                cycle += 1
            self.walk_Final_Pose()
            self.first_Leg_Is_Right_Leg = True
            if strong_kick == True:
                if kick_by_Right == True:
                    self.simulateMotion(name = 'Soccer_Kick_Forward_Right_Leg')
                else:
                    self.simulateMotion(name = 'Soccer_Kick_Forward_Left_Leg')
            else:
                self.kick( first_Leg_Is_Right_Leg=kick_by_Right, small = small_kick)
            #self.local.coord_odometry[0] += dist * math.cos(napravl)
            #self.local.coord_odometry[1] += dist * math.sin(napravl)
            #self.local.coordinate_record(odometry = True)
            self.head_Return(old_neck_pan, old_neck_tilt)
        return True

    def far_distance_ball_approach(self, ball_coord):
        old_neck_pan, old_neck_tilt = self.head_Up()
        self.local.correct_yaw_in_pf()
        ball_Approach(self, self.local, self.glob, ball_coord)
        self.head_Return(old_neck_pan, old_neck_tilt)

    def far_distance_plan_approach(self, ball_coord, target_yaw, stop_Over = False):
        dest = []
        centers = []
        price = 1000
        for i in range(5):
                for j in range(2):
                    target_x = ball_coord[0] - (0.21 + j * 0.05) * math.cos(target_yaw - 0.8 + i * 0.4)
                    target_y = ball_coord[1] - (0.21 + j * 0.05) * math.sin(target_yaw - 0.8 + i * 0.4)
                    target_coord = [target_x, target_y, target_yaw]
                    dest1, centers1, number_Of_Cycles = self.p.path_calc_optimum(self.glob.pf_coord, target_coord)
                    if i != 2: number_Of_Cycles += 50
                    if number_Of_Cycles <= price:
                        dest = dest1
                        centers = centers1
                        price = number_Of_Cycles
        #target_x = ball_coord[0] - 0.26 * math.cos(target_yaw)
        #target_y = ball_coord[1] - 0.26 * math.sin(target_yaw)
        #target_coord = [target_x, target_y, target_yaw]
        #dest, centers, price = self.p.path_calc_optimum(self.glob.pf_coord, target_coord)
        if stop_Over: price += 100
        start_yaw = self.glob.pf_coord[2]  #self.imu_body_yaw()
        if len(dest) == 0: return False
        old_neck_pan, old_neck_tilt = self.head_Up()
        self.local.correct_yaw_in_pf()
        sideLength = 0
        stepLength_old = 0
        acceleration = False
        deceleration = False
        self.walk_Initial_Pose()
# initial arc
        dest_yaw = self.p.coord2yaw(dest[1][0] - dest[0][0], dest[1][1] - dest[0][1] )
        x1, y1, x2, y2, cx, cy, R, CW = centers[0]
        delta_yaw = self.p.delta_yaw(start_yaw, dest_yaw, CW)
        #self.logger.debug('delta_yaw:' + str(delta_yaw) + ' CW:' + str(CW))
        number_Of_Cycles = math.ceil(abs(delta_yaw / 0.2))
        while True:
            delta_yaw_step = delta_yaw / number_Of_Cycles
            #self.logger.debug('R =' + str(R) + ' delta_yaw_step =' + str(delta_yaw_step) )
            stepLength = R * abs(delta_yaw_step) * 1000 * 64 / self.cycle_step_yield * 1.1
            if stepLength <= 64: break
            else: number_Of_Cycles += 1
        if stepLength - stepLength_old > 22 :
            acceleration = True
            number_Of_Cycles += 1
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            if acceleration:
                if cycle == 0: stepLength1 = stepLength / 3
                if cycle == 1: stepLength1 = stepLength * 2 / 3
            #self.refresh_Orientation()
            rotation = start_yaw + delta_yaw_step * (cycle + 1) - self.imu_body_yaw()
            rotation = self.normalize_rotation(rotation)
            self.walk_Cycle(stepLength1, sideLength, rotation, cycle, number_Of_Cycles+1)
            #self.logger.debug('stepLength1 =' + str(stepLength1) + 'rotation =' + str(rotation ) + 'cycle =' + str(cycle)  + 'number_Of_Cycles =' + str(number_Of_Cycles))
        stepLength_old = stepLength
        acceleration = False
# 1-st straight segment 
        L = math.sqrt((dest[1][0] - dest[0][0])**2 + (dest[1][1] - dest[0][1])**2)
        number_Of_Cycles = math.ceil(abs(L * 1000 / self.cycle_step_yield))
        stepLength = L * 1000 / number_Of_Cycles * 64 / self.cycle_step_yield
        if stepLength - stepLength_old > 22 :
            acceleration = True
            number_Of_Cycles += 1
        if not stop_Over:
            for cycle in range(number_Of_Cycles):
                stepLength1 = stepLength
                if acceleration:
                    if cycle == 0: stepLength1 = stepLength / 3
                    if cycle == 1: stepLength1 = stepLength * 2 / 3
                #self.refresh_Orientation()
                rotation = dest_yaw - self.imu_body_yaw()
                rotation = self.normalize_rotation(rotation)
                self.walk_Cycle(stepLength1, sideLength, rotation, cycle + 1, number_Of_Cycles+2)
            stepLength_old = stepLength
            acceleration = False
            for i in range(len(centers)-2):
                start_yaw = dest_yaw   #self.imu_body_yaw()
                dest_yaw = self.p.coord2yaw(dest[2*i+3][0] - dest[2*i+2][0], dest[2*i+3][1] - dest[2*i+2][1])
                x1, y1, x2, y2, cx, cy, R, CW = centers[i+1]
                delta_yaw = self.p.delta_yaw(start_yaw, dest_yaw, CW)
                number_Of_Cycles = math.ceil(abs(delta_yaw / 0.2))
                while True:
                    delta_yaw_step = delta_yaw / number_Of_Cycles
                    stepLength = R * abs(delta_yaw_step) * 1000 * 64 / self.cycle_step_yield * 1.1
                    if stepLength <= 64: break
                    else: number_Of_Cycles += 1
                if stepLength - stepLength_old > 22 :
                    acceleration = True
                    number_Of_Cycles += 1
                for cycle in range(number_Of_Cycles):
                    stepLength1 = stepLength
                    if acceleration:
                        if cycle == 0: stepLength1 = stepLength / 3
                        if cycle == 1: stepLength1 = stepLength * 2 / 3
                    #self.refresh_Orientation()
                    rotation = start_yaw + delta_yaw_step * (cycle + 1) - self.imu_body_yaw()
                    rotation = self.normalize_rotation(rotation)
                    if price < 100:
                        self.walk_Cycle(stepLength1, sideLength, rotation, cycle+1, number_Of_Cycles+2)
                    else:
                        self.walk_Cycle(stepLength1, sideLength, rotation, cycle+1, number_Of_Cycles+1)
                stepLength_old = stepLength
                acceleration = False
                if price >= 100: break
                L = math.sqrt((dest[2*i+3][0] - dest[2*i+2][0])**2 + (dest[2*i+3][1] - dest[2*i+2][1])**2)
                number_Of_Cycles = math.ceil(abs(L * 1000 / self.cycle_step_yield))
                stepLength = L * 1000 / number_Of_Cycles * 64 / self.cycle_step_yield
                if stepLength - stepLength_old > 22 :
                    acceleration = True
                    number_Of_Cycles += 1
                for cycle in range(number_Of_Cycles):
                    stepLength1 = stepLength
                    if acceleration:
                        if cycle == 0: stepLength1 = stepLength / 3
                        if cycle == 1: stepLength1 = stepLength * 2 / 3
                    #self.refresh_Orientation()
                    rotation = dest_yaw - self.imu_body_yaw()
                    rotation = self.normalize_rotation(rotation)
                    self.walk_Cycle(stepLength1, sideLength, rotation, cycle + 1, number_Of_Cycles+2)
                stepLength_old = stepLength
                acceleration = False
            if price < 100:
                start_yaw = dest_yaw   #self.imu_body_yaw()
                dest_yaw = target_yaw
                x1, y1, x2, y2, cx, cy, R, CW = centers[len(centers)-1]
                delta_yaw = self.p.delta_yaw(start_yaw, dest_yaw, CW)
                number_Of_Cycles = math.ceil(abs(delta_yaw / 0.2))
                while True:
                    delta_yaw_step = delta_yaw / number_Of_Cycles
                    stepLength = R * abs(delta_yaw_step) * 1000 * 64 / self.cycle_step_yield * 1.1
                    if stepLength <= 64: break
                    else: number_Of_Cycles += 1
                if stepLength - stepLength_old > 22 :
                    acceleration = True
                    number_Of_Cycles += 1
                if stepLength > 15:
                    deceleration = True
                    number_Of_Cycles += 1
                for cycle in range(number_Of_Cycles):
                    stepLength1 = stepLength
                    if acceleration:
                        if cycle == 0: stepLength1 = stepLength / 3
                        if cycle == 1: stepLength1 = stepLength * 2 / 3
                    if deceleration:
                        if cycle == number_Of_Cycles - 1: stepLength1 = stepLength / 3
                        if cycle == number_Of_Cycles - 2: stepLength1 = stepLength * 2 / 3
                    #self.refresh_Orientation()
                    rotation = start_yaw + delta_yaw_step * (cycle + 1) - self.imu_body_yaw()
                    rotation = self.normalize_rotation(rotation)
                    self.walk_Cycle(stepLength1, sideLength, rotation, cycle + 1, number_Of_Cycles + 1)
            # Adjustment of yaw position
            number_Of_Cycles = 4
            stepLength = 0
            for cycle in range(1, number_Of_Cycles+1, 1):
                #self.refresh_Orientation()
                rotation = target_yaw - self.imu_body_yaw()
                rotation = self.normalize_rotation(rotation)
                self.walk_Cycle(stepLength, sideLength, rotation, cycle, number_Of_Cycles+1)
        else:
            number_Of_Cycles = math.ceil(number_Of_Cycles/2)
            if stepLength > 15:
                    deceleration = True
                    number_Of_Cycles += 1
            else: deceleration = False
            for cycle in range(1, number_Of_Cycles + 1, 1):
                stepLength1 = stepLength
                if acceleration:
                    if cycle == 0: stepLength1 = stepLength / 3
                    if cycle == 1: stepLength1 = stepLength * 2 / 3
                #self.refresh_Orientation()
                rotation = dest_yaw - self.imu_body_yaw()
                rotation = self.normalize_rotation(rotation)
                if deceleration:
                    if cycle == number_Of_Cycles: stepLength1 = stepLength / 3
                    if cycle == number_Of_Cycles - 1: stepLength1 = stepLength * 2 / 3
                self.walk_Cycle(stepLength1, sideLength, rotation, cycle, number_Of_Cycles + 1)
        self.walk_Final_Pose()
        self.head_Return(old_neck_pan, old_neck_tilt)
        return True

 
if __name__=="__main__":
    print('This is not main module!')


