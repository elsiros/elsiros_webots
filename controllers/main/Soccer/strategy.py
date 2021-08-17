"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of Azer Babaev.
The module is designed for strategy of soccer game by forward and goalkeeper.
"""
import sys
import os
import math
import json
import time
from . import utility



class GoalKeeper:
    """
    class GoalKeeper is designed to define goalkeeper's play according to style developed by 
    Matvei Ivaschenko - a student of Phystech Lyceum in 2020.
    Idea of style was in dividing of home half of shoccer field to 8 sectors according to distance 
    from home goals. When ball is in 4 sectors closest to goals A1, A2, A3, A4 goalkeeper attacks ball with 
    purpose transfer it to side of opponent. When ball is in 4 sectors B1, B2, B3, B4 which are in longer
    distance from goals goalkeeper just slide to better position from current without attempt 
    to attack ball.
    In case if ball didn't go longer than 1m after kick of goalkeeper, he will undertake another attempt
    up to 10 times in total.
    In case if ball goes to distance longer than 1m or ball can't be seen by goalkeeper then goalkeeper
    returns to center of goals. 
    """
    def __init__(self, motion, local, glob):
        self.motion = motion
        self.local = local
        self.glob = glob
        self.direction_To_Guest = 0

    def turn_Face_To_Guest(self):
        """
        The method is designed to define kick direction and load it into self.direction_To_Guest
        direction is measured in radians of yaw. 
        After definition of kick direction robot turns to this direction.
        In case if robots' own coordinate self.glob.pf_coord shows lication on own half of field
        i.e. self.glob.pf_coord[0] < 0 then direction of shooting is 0
        if robots' x coordinate > 0.8 and abs(y coordinate) > 0.6 then direction of kick is to center
        of opponents' goals.
        if robots' x < 1.5 and abs(y) < 0.25 kick direction will be to corner of opponents' goals, 
        with left or right corner is defined randomly.
        in all other positions of robot kick direction is defined as ditection to target point with
        coordinates x = 0, y = 2.8
        """
        if self.glob.pf_coord[0] < 0:
            self.motion.turn_To_Course(0)
            self.direction_To_Guest = 0
            return
        elif self.glob.pf_coord[0] > 0.8 and abs(self.glob.pf_coord[1]) > 0.6:
            self.direction_To_Guest = math.atan(-self.glob.pf_coord[1]/(1.8-self.glob.pf_coord[0]))
            self.motion.turn_To_Course(self.direction_To_Guest)
        elif self.glob.pf_coord[0] < 1.5 and abs(self.glob.pf_coord[1]) < 0.25:
            if (1.8-self.glob.ball_coord[0]) == 0: self.direction_To_Guest = 0
            else: self.direction_To_Guest = math.atan((0.4* (round(utility.random(),0)*2 - 1)-
                                                       self.glob.ball_coord[1])/(1.8-self.glob.ball_coord[0]))
            self.motion.turn_To_Course(self.direction_To_Guest)
            return
        else:
            self.direction_To_Guest = math.atan(-self.glob.pf_coord[1]/(2.8-self.glob.pf_coord[0]))
            self.motion.turn_To_Course(self.direction_To_Guest)

    def goto_Center(self):                      #Function for reterning to center position
        """
        Goalkeeper returns to duty position 0.4m in front of own goals.
        before returning robot checks trustability of localization. If localization is poor then robot undertake 
        special motions by head and by turning to goals with purpose to improve localization.
        In case if distance to duty position is more than 0.5m then far_distance_plan_approach will be used,
        else near_distance_omni_motion will be used.
        after returning to duty position robot turns to kick direction, for which yaw=0 in front of own goals.

        """
        print('Function for reterning to center position')
        if self.local.coordinate_trust_estimation() < 0.5: self.motion.localisation_Motion()
        player_X_m = self.glob.pf_coord[0]
        player_Y_m = self.glob.pf_coord[1]
        duty_position_x = - self.glob.landmarks['FIELD_LENGTH']/2 + 0.4
        distance_to_target = math.sqrt((duty_position_x -player_X_m)**2 + (0 - player_Y_m)**2 )
        if distance_to_target > 0.5 :
            target_in_front_of_duty_position = [duty_position_x + 0.15, 0]
            if distance_to_target > 1: stop_Over = True
            else: stop_Over = False
            self.motion.far_distance_plan_approach(target_in_front_of_duty_position, self.direction_To_Guest, stop_Over = stop_Over)
        else:
            if (duty_position_x -player_X_m)==0:
                alpha = math.copysign(math.pi/2, (0 - player_Y_m) )
            else:
                if (duty_position_x - player_X_m)> 0: alpha = math.atan((0 - player_Y_m)/(duty_position_x -player_X_m))
                else: alpha = math.atan((0 - player_Y_m)/(duty_position_x - player_X_m)) + math.pi
            napravl = alpha - self.motion.imu_body_yaw()
            dist_mm = distance_to_target * 1000
            self.motion.near_distance_omni_motion(dist_mm, napravl)
        self.turn_Face_To_Guest()

    def find_Ball(self):
        """
        Before using motion method seek_Ball_In_Pose goalkeeper define usage of method in quick mode or in accurate mode.
        In case if localization is trustable quick mode is used means fast_Reaction_On=True. 
        seek_Ball_In_Pose method moves head of robot to 15 positions covering all visible areas in front and in sides of robot.
        this way seeking of ball is not single task. Robot improves localization through observing localization markers on 
        obtained pictures. 
        In case if fast_Reaction_On=True then observation of surroundings will be interrupted as soon as ball appear in visible 
        sector. Speed of ball is also detected. 
        """
        fast_Reaction_On=True
        if self.local.coordinate_trust_estimation() < 0.5: fast_Reaction_On = False
        if self.glob.ball_coord[0] <= 0: fast_Reaction_On=True
        success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On=fast_Reaction_On)
        #print ( 'dist = ', dist, 'napravl =', napravl)
        return success_Code, dist, napravl, speed

    def scenario_A1(self, dist, napravl):#The robot knock out the ball to the side of the opponent
        """
        This method is activated if goalkeeper finds ball at distance less than 0.7m and relative direction
        from 0 to math.pi/4. Supposed that goalkeeper stands on duty position faced to opponents' goals before seeking ball.
        usage: 
            None: self.scenario_A1(float:dist, float: napravl)
                dist -      distance to ball from goalkeeper in meters
                napravl -   relative direction to ball from goalkeeper in radians 

        method undertake 10 attempts to kick off ball to opponents side. In case of successful attempt - ball goes 1m away from 
        goalkeeper - goalkeeper returns to duty position in front of own goals. Otherwise attempts are continued up to 10 times.
        """
        print('The robot knock out the ball to the side of the opponent')
        for i in range(10):
            if dist > 0.5 :
                if dist > 1: stop_Over = True
                else: stop_Over = False
                self.motion.far_distance_plan_approach(self.glob.ball_coord, self.direction_To_Guest, stop_Over = stop_Over)
            self.turn_Face_To_Guest()
            success_Code = self.motion.near_distance_ball_approach_and_kick(self.direction_To_Guest)
            if success_Code == False and self.motion.falling_Flag != 0: return
            if success_Code == False : break
            success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = False)
            if dist > 1 : break
        target_course1 = self.glob.pf_coord[2] +math.pi
        self.motion.turn_To_Course(target_course1)
        self.goto_Center()

    def scenario_A2(self, dist, napravl):#The robot knock out the ball to the side of the opponent
        """
        This method is activated if goalkeeper finds ball at distance less than 0.7m and relative direction
        from math.pi/4 to math.pi/2. Supposed that goalkeeper stands on duty position faced to opponents' goals before seeking ball.
        usage: 
            None: self.scenario_A1(float:dist, float: napravl)
                dist -      distance to ball from goalkeeper in meters
                napravl -   relative direction to ball from goalkeeper in radians 

        method undertake 10 attempts to kick off ball to opponents side. In case of successful attempt - ball goes 1m away from 
        goalkeeper - goalkeeper returns to duty position in front of own goals. Otherwise attempts are continued up to 10 times.
        """
        print('The robot knock out the ball to the side of the opponent')
        self.scenario_A1( dist, napravl)

    def scenario_A3(self, dist, napravl):#The robot knock out the ball to the side of the opponent
        """
        This method is activated if goalkeeper finds ball at distance less than 0.7m and relative direction
        from 0 to -math.pi/4. Supposed that goalkeeper stands on duty position faced to opponents' goals before seeking ball.
        usage: 
            None: self.scenario_A1(float:dist, float: napravl)
                dist -      distance to ball from goalkeeper in meters
                napravl -   relative direction to ball from goalkeeper in radians 

        method undertake 10 attempts to kick off ball to opponents side. In case of successful attempt - ball goes 1m away from 
        goalkeeper - goalkeeper returns to duty position in front of own goals. Otherwise attempts are continued up to 10 times.
        """
        print('The robot knock out the ball to the side of the opponent')
        self.scenario_A1( dist, napravl)

    def scenario_A4(self, dist, napravl):#The robot knock out the ball to the side of the opponent
        """
        This method is activated if goalkeeper finds ball at distance less than 0.7m and relative direction
        from -math.pi/4 to -math.pi/2. Supposed that goalkeeper stands on duty position faced to opponents' goals before seeking ball.
        usage: 
            None: self.scenario_A1(float:dist, float: napravl)
                dist -      distance to ball from goalkeeper in meters
                napravl -   relative direction to ball from goalkeeper in radians 

        method undertake 10 attempts to kick off ball to opponents side. In case of successful attempt - ball goes 1m away from 
        goalkeeper - goalkeeper returns to duty position in front of own goals. Otherwise attempts are continued up to 10 times.
        """
        print('The robot knock out the ball to the side of the opponent')
        self.scenario_A1( dist, napravl)

    def scenario_B1(self):#the robot moves to the left and stands on the same axis as the ball and the opponents' goal
        """
        This method is activated if goalkeeper finds ball at distance more than 0.7m and less than half of length of field
        and relative direction from 0 to math.pi/4. 
        Supposed that goalkeeper stands on duty position faced to opponents' goals before seeking ball.
        method undertake to slide robot sideways to same Y coordinate as balls' Y coordinate. In case if balls' Y coordinate
        abs value is more than 0.4m robots maximum Y coordinate abs value will be 0.4m
        After sliding sideways robot undertake turning to 0 direction
        """
        print('the robot moves to the left 4 steps')
        if self.glob.ball_coord[1] > self.glob.pf_coord[1]:
            if self.glob.ball_coord[1] > 0.4: 
                if self.glob.pf_coord[1] < 0.4:
                    self.motion.near_distance_omni_motion( 1000*(0.4 - self.glob.pf_coord[1]), math.pi/2)
            else:
                self.motion.near_distance_omni_motion( 1000*(self.glob.ball_coord[1] - self.glob.pf_coord[1]), math.pi/2)
        self.turn_Face_To_Guest()

    def scenario_B2(self):#the robot moves to the left and stands on the same axis as the ball and the opponents' goal
        """
        This method is activated if goalkeeper finds ball at distance more than 0.7m and less than half of length of field
        and relative direction from 0 to math.pi/4. 
        Supposed that goalkeeper stands on duty position faced to opponents' goals before seeking ball.
        method undertake to slide robot sideways to same Y coordinate as balls' Y coordinate. In case if balls' Y coordinate
        abs value is more than 0.4m robots maximum Y coordinate abs value will be 0.4m
        After sliding sideways robot undertake turning to 0 direction
        """
        print('the robot moves to the left 4 steps')
        self.scenario_B1()

    def scenario_B3(self):#the robot moves to the right and stands on the same axis as the ball and the opponents' goal
        """
        This method is activated if goalkeeper finds ball at distance more than 0.7m and less than half of length of field
        and relative direction from 0 to math.pi/4. 
        Supposed that goalkeeper stands on duty position faced to opponents' goals before seeking ball.
        method undertake to slide robot sideways to same Y coordinate as balls' Y coordinate. In case if balls' Y coordinate
        abs value is more than 0.4m robots maximum Y coordinate abs value will be 0.4m
        After sliding sideways robot undertake turning to 0 direction
        """
        print('the robot moves to the right 4 steps')
        #self.motion.first_Leg_Is_Right_Leg = True
        #self.motion.near_distance_omni_motion( 110, -math.pi/2)
        if self.glob.ball_coord[1] < self.glob.pf_coord[1]:
            if self.glob.ball_coord[1] < -0.4: 
                if self.glob.pf_coord[1] > -0.4:
                    self.motion.near_distance_omni_motion( 1000*(0.4 + self.glob.pf_coord[1]), -math.pi/2)
            else:
                self.motion.near_distance_omni_motion( 1000*(-self.glob.ball_coord[1] + self.glob.pf_coord[1]), -math.pi/2)
        self.turn_Face_To_Guest()

    def scenario_B4(self):#the robot moves to the right and stands on the same axis as the ball and the opponents' goal
        """
        This method is activated if goalkeeper finds ball at distance more than 0.7m and less than half of length of field
        and relative direction from 0 to math.pi/4. 
        Supposed that goalkeeper stands on duty position faced to opponents' goals before seeking ball.
        method undertake to slide robot sideways to same Y coordinate as balls' Y coordinate. In case if balls' Y coordinate
        abs value is more than 0.4m robots maximum Y coordinate abs value will be 0.4m
        After sliding sideways robot undertake turning to 0 direction
        """
        print('the robot moves to the right 4 steps')
        self.scenario_B3()

class Forward:
    """
    The class Forward is designed for definition of strategy of play for 'forward' role of player 
    in year 2020.
    usage:
        Forward(object: motion, object: lical, object: glob)
    """
    def __init__(self, motion, local, glob):
        self.motion = motion
        self.local = local
        self.glob = glob
        self.direction_To_Guest = 0

    def dir_To_Guest(self):
        """
        The method is designed to define kick direction and load it into self.direction_To_Guest,
        direction is measured in radians of yaw. 
        In case if robots' own coordinate self.glob.pf_coord shows lication on own half of field
        i.e. self.glob.pf_coord[0] < 0 then direction of shooting is 0
        if robots' x coordinate > 0.8 and abs(y coordinate) > 0.6 then direction of kick is to center
        of opponents' goals.
        if robots' x < 1.5 and abs(y) < 0.25 kick direction will be to corner of opponents' goals, 
        with left or right corner is defined randomly.
        in all other positions of robot kick direction is defined as ditection to target point with
        coordinates x = 0, y = 2.8
        returns float: self.direction_To_Guest
        """
        if self.glob.ball_coord[0] < 0:
            self.direction_To_Guest = 0
        elif self.glob.ball_coord[0] > 0.8 and abs(self.glob.ball_coord[1]) > 0.6:
            self.direction_To_Guest = math.atan(-self.glob.ball_coord[1]/(1.8-self.glob.ball_coord[0]))
        elif self.glob.ball_coord[0] < 1.5 and abs(self.glob.ball_coord[1]) < 0.25:
            if (1.8-self.glob.ball_coord[0]) == 0: self.direction_To_Guest = 0
            else:
                if abs(self.glob.ball_coord[1]) > 0.2:
                    self.direction_To_Guest = math.atan((math.copysign(0.2, self.glob.ball_coord[1])-
                                                       self.glob.ball_coord[1])/(1.8-self.glob.ball_coord[0]))
                else:
                    self.direction_To_Guest = math.atan((0.2* (round(utility.random(),0)*2 - 1)-
                                                       self.glob.ball_coord[1])/(1.8-self.glob.ball_coord[0]))
        else:
            self.direction_To_Guest = math.atan(-self.glob.pf_coord[1]/(2.8-self.glob.pf_coord[0]))
        return self.direction_To_Guest

    def turn_Face_To_Guest(self):
        self.dir_To_Guest()
        self.motion.turn_To_Course(self.direction_To_Guest)

class Forward_Vector_Matrix:
    def __init__(self, motion, local, glob):
        self.motion = motion
        self.local = local
        self.glob = glob
        self.direction_To_Guest = 0
        self.kick_Power = 1

    def dir_To_Guest(self):
        if abs(self.glob.ball_coord[0])  >  self.glob.landmarks["FIELD_LENGTH"] / 2:
            ball_x = math.copysign(self.glob.landmarks["FIELD_LENGTH"] / 2, self.glob.ball_coord[0])
        else: ball_x = self.glob.ball_coord[0]
        if abs(self.glob.ball_coord[1])  >  self.glob.landmarks["FIELD_WIDTH"] / 2:
            ball_y = math.copysign(self.glob.landmarks["FIELD_WIDTH"] / 2, self.glob.ball_coord[1])
        else: ball_y = self.glob.ball_coord[1]
        col = math.floor((ball_x + self.glob.landmarks["FIELD_LENGTH"] / 2) / (self.glob.landmarks["FIELD_LENGTH"] / self.glob.COLUMNS))
        row = math.floor((- ball_y + self.glob.landmarks["FIELD_WIDTH"] / 2) / (self.glob.landmarks["FIELD_WIDTH"] / self.glob.ROWS))
        if col >= self.glob.COLUMNS : col = self.glob.COLUMNS - 1
        if row >= self.glob.ROWS : row = self.glob.ROWS -1
        self.direction_To_Guest = self.glob.strategy_data[(col * self.glob.ROWS + row) * 2 + 1] / 40
        self.kick_Power = self.glob.strategy_data[(col * self.glob.ROWS + row) * 2]
        #print('direction_To_Guest = ', math.degrees(self.direction_To_Guest))
        return row, col

    def turn_Face_To_Guest(self):
        self.dir_To_Guest()
        self.motion.turn_To_Course(self.direction_To_Guest)

class Player():
    def __init__(self, role, second_pressed_button, glob, motion, local):
        self.role = role   #'goalkeeper', 'penalty_Goalkeeper', 'forward', 'penalty_Shooter'
        self.second_pressed_button = second_pressed_button
        self.glob = glob
        self.motion = motion
        self.local = local
        self.g = None
        self.f = None

    def play_game(self):
        if self.role == 'goalkeeper': self.goalkeeper_main_cycle()
        if self.role == 'goalkeeper_old_style': self.goalkeeper_old_style_main_cycle()
        if self.role == 'penalty_Goalkeeper': self.penalty_Goalkeeper_main_cycle()
        if self.role == 'forward': self.forward_main_cycle(self.second_pressed_button)
        if self.role == 'forward_old_style': self.forward_old_style_main_cycle(self.second_pressed_button)
        if self.role == 'penalty_Shooter': self.penalty_Shooter_main_cycle()
        if self.role == 'run_test': self.run_test_main_cycle(self.second_pressed_button)
        if self.role == 'spot_walk': self.spot_walk_main_cycle(self.second_pressed_button)
        if self.role == 'rotation_test': self.rotation_test_main_cycle()
        if self.role == 'sidestep_test': self.sidestep_test_main_cycle()
        if self.role == 'dance': self.dance_main_cycle()
        if self.glob.SIMULATION != 2:
            self.motion.sim_Stop()
            self.motion.sim_Disable()

    def rotation_test_main_cycle(self, pressed_button):
        number_Of_Cycles = 20
        stepLength = 0
        sideLength = 0
        if pressed_button == 6: rotation = 0.23 # 0.483
        else: rotation = -0.23
        #self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = -1
        else: invert = 1
        self.motion.walk_Initial_Pose()
        for cycle in range(number_Of_Cycles):
            self.motion.walk_Cycle(stepLength,sideLength, rotation, cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()
        self.motion.refresh_Orientation()
        print('self.motion.imu_body_yaw() =', self.motion.imu_body_yaw())

    def spot_walk_main_cycle(self, pressed_button):
        if pressed_button == 2 or pressed_button ==3 :
            self.rotation_test_main_cycle(pressed_button)
            return
        self.run_test_main_cycle(1, stepLength = 0)

    def run_test_main_cycle(self, pressed_button, stepLength = 64):
        if pressed_button == 2 or pressed_button ==3 :
            self.sidestep_test_main_cycle(pressed_button)
            return
        if pressed_button == 5 or pressed_button ==6 :
            self.rotation_test_main_cycle(pressed_button)
            return
        #stepLength = 64
        if pressed_button == 9: stepLength = 0
        number_Of_Cycles = 20
        if pressed_button == 1: number_Of_Cycles = 10
        sideLength = 0
        #self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = -1
        else: invert = 1
        self.motion.walk_Initial_Pose()
        number_Of_Cycles += 1
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            if cycle ==0 : stepLength1 = stepLength/3
            if cycle ==1 : stepLength1 = stepLength/3 * 2
            self.motion.refresh_Orientation()
            rotation = 0 + invert * self.motion.imu_body_yaw() * 1.2
            if rotation > 0: rotation *= 1.5
            rotation = self.motion.normalize_rotation(rotation)
            #rotation = 0
            self.motion.walk_Cycle(stepLength1,sideLength, rotation,cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()

    def sidestep_test_main_cycle(self, pressed_button):
        number_Of_Cycles = 20
        stepLength = 0 #64
        sideLength = 20
        if pressed_button == 3:
            self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = -1
        else: invert = 1
        self.motion.walk_Initial_Pose()
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            #if cycle ==0 : stepLength1 = stepLength/3
            #if cycle ==1 : stepLength1 = stepLength/3 * 2
            self.motion.refresh_Orientation()
            rotation = 0 + invert * self.motion.imu_body_yaw() * 1.0
            rotation = self.motion.normalize_rotation(rotation)
            #rotation = 0
            self.motion.walk_Cycle(stepLength1,sideLength, rotation,cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()

    def norm_yaw(self, yaw):
        yaw %= 2 * math.pi
        if yaw > math.pi:  yaw -= 2* math.pi
        if yaw < -math.pi: yaw += 2* math.pi
        return yaw

    def forward_main_cycle(self, pressed_button):
        second_player_timer = self.motion.game_time()
        self.f = Forward_Vector_Matrix(self.motion, self.local, self.glob)
        while (True):
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()
            success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True)
            if self.glob.SIMULATION == 2 and self.glob.wifi_params['WIFI_IS_ON']: self.local.report_to_WIFI()
            if pressed_button == 4 and (self.motion.game_time() - second_player_timer) < 10 : continue
            self.f.dir_To_Guest()
            print('direction_To_Guest = ', math.degrees(self.f.direction_To_Guest), 'degrees')
            print('coord =', self.glob.pf_coord, 'ball =', self.glob.ball_coord)
            if dist == 0 and success_Code == False:
                self.motion.turn_To_Course(self.glob.pf_coord[2]+ 2 * math.pi / 3)
                continue
            player_from_ball_yaw = self.motion.p.coord2yaw(self.glob.pf_coord[0] - self.glob.ball_coord[0], self.glob.pf_coord[1] - self.glob.ball_coord[1]) - self.f.direction_To_Guest
            player_from_ball_yaw = self.norm_yaw(player_from_ball_yaw)
            player_in_front_of_ball = -math.pi/2 < player_from_ball_yaw < math.pi/2
            player_in_fast_kick_position = (player_from_ball_yaw > 2 or player_from_ball_yaw < -2) and dist < 0.6 
            if (dist > 0.35  or player_in_front_of_ball) and not player_in_fast_kick_position:
                if dist > 1: stop_Over = True
                else: stop_Over = False
                self.motion.far_distance_plan_approach(self.glob.ball_coord, self.f.direction_To_Guest, stop_Over = stop_Over)
                #self.f.turn_Face_To_Guest()
                continue
            self.motion.turn_To_Course(self.f.direction_To_Guest)
            small_kick = False
            if self.f.kick_Power > 1: small_kick = True
            success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False, small_kick = small_kick)
        self.motion.play_Soft_Motion_Slot(name = 'Initial_Pose')

    def forward_old_style_main_cycle(self, pressed_button):
        second_player_timer = self.motion.game_time()
        self.f = Forward(self.motion, self.local, self.glob)
        first_shoot = False
        while (True):
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()
            success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True)
            if pressed_button == 4 and (self.motion.game_time() - second_player_timer) < 10 : continue
            if dist == 0 and success_Code == False:
                self.motion.turn_To_Course(self.glob.pf_coord[2]+ 2 * math.pi / 3)
                continue
            if dist > 0.5  or self.glob.pf_coord[0] > self.glob.ball_coord[0] :
                self.motion.far_distance_ball_approach(self.glob.ball_coord)
                self.f.turn_Face_To_Guest()
                continue
            if first_shoot == False:
                success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False)
                #motion.near_distance_omni_motion(500, 0)
                first_shoot = True
                if self.motion.falling_Flag != 0: continue
            else:
                success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False)

    def goalkeeper_main_cycle(self):
        def ball_position_is_dangerous(row, col):
            danger = False
            danger = (col <= (round(self.glob.COLUMNS / 3) - 1))
            if ((row <= (round(self.glob.ROWS / 3) - 1) or row >= round(self.glob.ROWS * 2 / 3)) and col == 0) or (col == 1 and (row == 0 or row == (self.glob.ROWS -1))):
               danger = False
            return danger
        second_player_timer = self.motion.game_time()
        self.f = Forward_Vector_Matrix(self.motion, self.local, self.glob)
        #self.motion.near_distance_omni_motion(400, 0)                    # get out from goal
        fast_Reaction_On = True
        while (True):
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()
            if self.glob.ball_coord[0] <= 0.15:
                success_Code, napravl, dist, speed =  self.motion.watch_Ball_In_Pose()
            else: 
                success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = fast_Reaction_On)
            if abs(speed[0]) > 0.02 and dist < 1 :                         # if dangerous tangential speed
                fast_Reaction_On = True
                if speed[0] > 0:
                    if self.glob.pf_coord[1] < 0.35:
                        self.motion.play_Soft_Motion_Slot(name ='PenaltyDefenceL')
                else:
                    if self.glob.pf_coord[1] > -0.35:
                        self.motion.play_Soft_Motion_Slot(name ='PenaltyDefenceR')
                self.motion.pause_in_ms(3000)
                continue
            if speed[1] < - 0.01 and dist < 1.5 :                          # if dangerous front speed
                fast_Reaction_On = True
                self.motion.play_Soft_Motion_Slot(name = 'PanaltyDefenceReady_Fast')
                self.motion.play_Soft_Motion_Slot(name = 'PenaltyDefenceF')
                self.motion.pause_in_ms(3000)
                self.motion.play_Soft_Motion_Slot(name = 'Get_Up_From_Defence')
                continue
            if (self.motion.game_time() - second_player_timer) < 10 : continue
            row, col = self.f.dir_To_Guest()
            #print('direction_To_Guest = ', math.degrees(self.f.direction_To_Guest), 'degrees')
            #print('goalkeeper coord =', self.glob.pf_coord, 'ball =', self.glob.ball_coord, 'row =', row, 'col =', col, 'ball_position_is_dangerous =', ball_position_is_dangerous(row,col))
            if dist == 0 and success_Code == False:
                print('goalkeeper turn_To_Course(pi*2/3)')
                self.motion.turn_To_Course(self.glob.pf_coord[2]+ 2 * math.pi / 3)
                continue
            if ball_position_is_dangerous(row, col):
                fast_Reaction_On = True
                player_from_ball_yaw = self.motion.p.coord2yaw(self.glob.pf_coord[0] - self.glob.ball_coord[0], self.glob.pf_coord[1] - self.glob.ball_coord[1]) - self.f.direction_To_Guest
                player_from_ball_yaw = self.norm_yaw(player_from_ball_yaw)
                player_in_front_of_ball = -math.pi/2 < player_from_ball_yaw < math.pi/2
                player_in_fast_kick_position = (player_from_ball_yaw > 2 or player_from_ball_yaw < -2) and dist < 0.6 
                if (dist > 0.35  or player_in_front_of_ball) and not player_in_fast_kick_position:
                    if dist > 1: stop_Over = True
                    else: stop_Over = False
                    print('goalkeeper far_distance_plan_approach')
                    self.motion.far_distance_plan_approach(self.glob.ball_coord, self.f.direction_To_Guest, stop_Over = stop_Over)
                    #self.f.turn_Face_To_Guest()
                    continue
                print('goalkeeper turn_To_Course(direction_To_Guest)')
                self.motion.turn_To_Course(self.f.direction_To_Guest)
                small_kick = False
                if self.f.kick_Power > 1: small_kick = True
                print('goalkeeper near_distance_ball_approach_and_kick')
                success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False, small_kick = small_kick)

            else:
                fast_Reaction_On = False
                duty_x_position =  min((-self.glob.landmarks['FIELD_LENGTH']/2 + 0.4),(self.glob.ball_coord[0]-self.glob.landmarks['FIELD_LENGTH']/2)/2)
                duty_y_position = self.glob.ball_coord[1] * (duty_x_position + self.glob.landmarks['FIELD_LENGTH']/2) / (self.glob.ball_coord[0] + self.glob.landmarks['FIELD_LENGTH']/2) 
                duty_distance = math.sqrt((duty_x_position - self.glob.pf_coord[0])**2 + (duty_y_position - self.glob.pf_coord[1])**2)
                #print('duty_x_position =', duty_x_position, 'duty_y_position =', duty_y_position)
                if duty_distance < 0.2 : continue
                elif duty_distance <  3: #   0.6 :
                    print('goalkeeper turn_To_Course(0)')
                    self.motion.turn_To_Course(0)
                    duty_direction = self.motion.p.coord2yaw(duty_x_position - self.glob.pf_coord[0], duty_y_position - self.glob.pf_coord[1])  
                    print('goalkeeper near_distance_omni_motion')
                    self.motion.near_distance_omni_motion(duty_distance * 1000, duty_direction)
                    print('goalkeeper turn_To_Course(0)')
                    self.motion.turn_To_Course(0)
                else:
                    self.motion.far_distance_plan_approach([duty_x_position + 0.25, duty_y_position], 0, stop_Over = False)
        self.motion.play_Soft_Motion_Slot(name = 'Initial_Pose')

    def goalkeeper_old_style_main_cycle(self):
        self.motion.near_distance_omni_motion(200, 0)                    # get out from goal
        self.g = GoalKeeper(self.motion, self.local, self.glob)
        while (True):
            dist = -1.0
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()
                self.g.turn_Face_To_Guest()
                #goto_Center()
            while(dist < 0):
                a, dist,napravl, speed = self.g.find_Ball()
                #uprint('speed = ', speed, 'dist  =', dist , 'napravl =', napravl)
                if abs(speed[0]) > 0.02 and dist < 1 :                         # if dangerous tangential speed
                    if speed[0] > 0:
                        if self.glob.pf_coord[1] < 0.35:
                            self.motion.play_Soft_Motion_Slot(name ='PenaltyDefenceL')
                    else:
                        if self.glob.pf_coord[1] > -0.35:
                            self.motion.play_Soft_Motion_Slot(name ='PenaltyDefenceR')
                    if self.glob.SIMULATION == 2: pyb.delay(3000)
                    else: self.motion.sim_Progress(3)
                    continue
                if speed[1] < - 0.01 and dist < 1.5 :                          # if dangerous front speed
                    self.motion.play_Soft_Motion_Slot(name = 'PanaltyDefenceReady_Fast')
                    self.motion.play_Soft_Motion_Slot(name = 'PenaltyDefenceF')
                    if self.glob.SIMULATION == 2: pyb.delay(8000)
                    else: self.motion.sim_Progress(3)
                    self.motion.play_Soft_Motion_Slot(name = 'Get_Up_From_Defence')

                if (dist == 0 and napravl == 0) or dist > 2.5:
                    position_limit_x1 = -self.glob.landmarks['FIELD_LENGTH']/2 - 0.05
                    position_limit_x2 = position_limit_x1 + 0.25
                    if position_limit_x1 < self.glob.pf_coord[0] < position_limit_x2 and -0.05 < self.glob.pf_coord[1] < 0.05: break
                    self.g.goto_Center()
                    break
                old_neck_pan, old_neck_tilt = self.motion.head_Up()
                if (dist <= 0.7         and 0 <= napravl <= math.pi/4):         self.g.scenario_A1( dist, napravl)
                if (dist <= 0.7         and math.pi/4 < napravl <= math.pi/2):  self.g.scenario_A2( dist, napravl)
                if (dist <= 0.7         and 0 >= napravl >= -math.pi/4):        self.g.scenario_A3( dist, napravl)
                if (dist <= 0.7         and -math.pi/4 > napravl >= -math.pi/2): self.g.scenario_A4( dist, napravl)
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (math.pi/18 <= napravl <= math.pi/4)): self.g.scenario_B1()
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (math.pi/4 < napravl <= math.pi/2)): self.g.scenario_B2()
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (-math.pi/18 >= napravl >= -math.pi/4)): self.g.scenario_B3()
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (-math.pi/4 > napravl >= -math.pi/2)): self.g.scenario_B4()

    def penalty_Shooter_main_cycle(self):
        self.f = Forward(self.motion, self.local, self.glob)
        first_shoot = True
        while (True):
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()
            success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True)
            self.f.dir_To_Guest()
            print('ball_coord = ', self.glob.ball_coord)
            print('direction_To_Guest = ', math.degrees(self.f.direction_To_Guest), 'degrees')
            if dist == 0 and success_Code == False:
                self.motion.turn_To_Course(self.glob.pf_coord[2]+ 2 * math.pi / 3)
                continue
            if dist > 0.35 or self.glob.pf_coord[0] - 0.2 > self.glob.ball_coord[0]:
                if dist > 1: stop_Over = True
                else: stop_Over = False
                self.motion.far_distance_plan_approach(self.glob.ball_coord, self.f.direction_To_Guest, stop_Over = stop_Over)
            kick_direction = self.f.direction_To_Guest
            self.motion.turn_To_Course(kick_direction)
            #if first_shoot:
            if self.motion.params['DRIBBLING'] == 1:
                success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True)
                if napravl > 0.2 : 
                    self.motion.near_distance_omni_motion(dist* math.cos(napravl)*1000/2, math.pi/2)
                if napravl < -0.2 : 
                    self.motion.near_distance_omni_motion(dist* math.cos(napravl)*1000, -math.pi/2)
                number_Of_Cycles = 15
                stepLength = 20
                sideLength = 0
                old_neck_pan, old_neck_tilt = self.motion.head_Up()
                self.motion.walk_Initial_Pose()
                for cycle in range(number_Of_Cycles):
                    stepLength1 = stepLength
                    #if cycle ==0 : stepLength1 = stepLength/4
                    #if cycle ==1 : stepLength1 = stepLength/2
                    self.motion.refresh_Orientation()
                    rotation = self.f.direction_To_Guest - self.motion.imu_body_yaw()
                    rotation = self.motion.normalize_rotation(rotation)
                    self.motion.walk_Cycle(stepLength1,sideLength, rotation,cycle, number_Of_Cycles)
                self.motion.walk_Final_Pose()
                self.motion.head_Return(old_neck_pan, old_neck_tilt)
            else:
                success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False)
            #self.motion.near_distance_omni_motion(700, 0)
            first_shoot = False
            #return
            if self.motion.falling_Flag != 0: continue
            #else:
            #    success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False)

    def penalty_Goalkeeper_main_cycle(self):
        self.g = GoalKeeper(self.motion, self.local, self.glob)
        self.glob.obstacleAvoidanceIsOn = False
        first_Get_Up = True
        while (True):
            dist = -1.0
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()
                self.g.turn_Face_To_Guest()
                if first_Get_Up:
                    first_Get_Up = False
                    self.g.goto_Center()
            while(dist < 0):
                a, napravl, dist, speed = self.motion.watch_Ball_In_Pose(penalty_Goalkeeper = True)
                #print('speed = ', speed, 'dist  =', dist , 'napravl =', napravl)
                if abs(speed[0]) > 0.002 and dist < 1 :                         # if dangerous tangential speed
                    if speed[0] > 0:
                        self.motion.play_Soft_Motion_Slot(name ='PenaltyDefenceL')
                    else:
                        self.motion.play_Soft_Motion_Slot(name ='PenaltyDefenceR')
                    continue
                if speed[1] < - 0.01 and dist < 1.5 :                          # if dangerous front speed
                    self.motion.play_Soft_Motion_Slot(name = 'PanaltyDefenceReady_Fast')
                    self.motion.play_Soft_Motion_Slot(name = 'PenaltyDefenceF')
                    self.motion.pause_in_ms(5000)
                    self.motion.play_Soft_Motion_Slot(name = 'Get_Up_From_Defence')

                if (dist == 0 and napravl == 0) or dist > 2.5:
                    #position_limit_x1 = -self.glob.landmarks['FIELD_LENGTH']/2 - 0.05
                    #position_limit_x2 = position_limit_x1 + 0.25
                    #if position_limit_x1 < self.glob.pf_coord[0] < position_limit_x2 and -0.05 < self.glob.pf_coord[1] < 0.05: break
                    #self.g.goto_Center()
                    #break
                    continue
                old_neck_pan, old_neck_tilt = self.motion.head_Up()
                if (dist <= 0.7         and 0 <= napravl <= math.pi/4):         self.g.scenario_A1( dist, napravl)
                if (dist <= 0.7         and math.pi/4 < napravl <= math.pi/2):  self.g.scenario_A2( dist, napravl)
                if (dist <= 0.7         and 0 >= napravl >= -math.pi/4):        self.g.scenario_A3( dist, napravl)
                if (dist <= 0.7         and -math.pi/4 > napravl >= -math.pi/2): self.g.scenario_A4( dist, napravl)
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (math.pi/18 <= napravl <= math.pi/4)): self.g.scenario_B1()
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (math.pi/4 < napravl <= math.pi/2)): self.g.scenario_B2()
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (-math.pi/18 >= napravl >= -math.pi/4)): self.g.scenario_B3()
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (-math.pi/4 > napravl >= -math.pi/2)): self.g.scenario_B4()
                self.motion.head_Return(old_neck_pan, old_neck_tilt)

    def dance_main_cycle(self):
        if self.glob.SIMULATION == 2:
            while True:
                successCode, u10 = self.motion.kondo.getUserParameter(10)
                #time.sleep(2)
                if successCode and u10 == 1:
                    self.motion.kondo.motionPlay(26)
                    for i in range(10):
                        self.motion.play_Soft_Motion_Slot( name = 'Dance_6_1')
                    self.motion.play_Soft_Motion_Slot( name = 'Dance_7-1')
                    self.motion.play_Soft_Motion_Slot( name = 'Dance_7-2')
                    for i in range(9):
                        self.motion.play_Soft_Motion_Slot( name = 'Dance_6_1')
                    self.motion.play_Soft_Motion_Slot( name = 'Dance_2')
                    for i in range(2):
                        self.motion.play_Soft_Motion_Slot( name = 'Dance_6_1')
                    self.motion.play_Soft_Motion_Slot( name = 'Dance_4')
                    self.motion.kondo.setUserParameter(10,0)
        else:
            for i in range(10):
                self.motion.play_Soft_Motion_Slot( name = 'Dance_6_1')
            self.motion.play_Soft_Motion_Slot( name = 'Dance_7')
            self.motion.play_Soft_Motion_Slot( name = 'Dance_2')
            self.motion.play_Soft_Motion_Slot( name = 'Dance_4')


if __name__=="__main__":
    pass