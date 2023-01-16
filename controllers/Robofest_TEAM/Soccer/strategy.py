"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of Azer Babaev.
The module is designed for strategy of soccer game for forward and goalkeeper.
"""
import sys
import os
import math
import json
import time
import logging
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
    def __init__(self, logger, motion, local, glob):
        self.logger = logger
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
        self.logger.info('Function for reterning to center position')
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
        self.logger.debug( 'dist = ' + str(dist) + 'napravl =' + str(napravl))
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
        self.logger.info('The robot knock out the ball to the side of the opponent')
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
        self.logger.info('The robot knock out the ball to the side of the opponent')
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
        self.logger.info('The robot knock out the ball to the side of the opponent')
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
        self.logger.info('The robot knock out the ball to the side of the opponent')
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
        self.logger.info('the robot moves to the left 4 steps')
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
        self.logger.info('the robot moves to the left 4 steps')
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
        self.logger.info('the robot moves to the right 4 steps')
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
        self.logger.info('the robot moves to the right 4 steps')
        self.scenario_B3()

class Forward:
    """
    The class Forward is designed for definition of strategy of play for 'forward' role of player 
    in year 2020.
    usage:
        Forward(object: motion, object: lical, object: glob)
    """
    def __init__(self, logger, motion, local, glob):
        self.logger = logger
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
    """
    The class Forward_Vector_Matrix is designed for definition of strategy of play for 'forward' role of player 
    in year 2021.
    Matrix is coded in file strategy_data.json  This file is readable and editable as well as
    normal text file. There is a dictionary with one key “strategy_data”. Value of key
    “strategy_data” is a list with default number of elements 234. Each element of list represents
    rectangular sector of soccer field with size 20cmX20cm. For each sector there assigned a vector
    representing yaw direction of shooting when ball is positioned in this sector. Power of shot is
    coded by attenuation value: 1 – standard power, 2 – power reduced 2 times, 3- power reduced 3 times.
    Each element of list is coded as follows: [column, row, power, yaw]. Soccer field is split to
    sectors in 13 rows and 18 columns.  Column 0 is near own goals, column 17 is near opposed goals.
    Row 0 is in positive Y coordinate, row 12 is in negative Y coordinate.
    Strategy data is imported from strategy_data.json file into self.glob.strategy_data list.
    usage:  
        Forward_Vector_Matrix(object: motion, object: local, object: glob)
    """
    def __init__(self, logger, motion, local, glob):
        self.logger = logger
        self.motion = motion
        self.local = local
        self.glob = glob
        self.direction_To_Guest = 0
        self.kick_Power = 1

    def dir_To_Guest(self):
        """
        The method is designed to define kick direction and load it into self.direction_To_Guest.
        Direction is measured in radians of yaw.
        usage:
            int: row, int: col = self.dir_To_Guest()
            row, col  -  row and column of matrix attributing rectangular sector of field where
            ball coorinate  self.glob.ball_coord fits.
        """
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
        self.logger.debug('direction_To_Guest = ' + str(math.degrees(self.direction_To_Guest)))
        return row, col

    def turn_Face_To_Guest(self):
        self.dir_To_Guest()
        self.motion.turn_To_Course(self.direction_To_Guest)

class Player():
    """
    class Player is designed for implementation of main cycle of player.
    Real robot have 3 programmable buttons. Combination of button pressing can transmit to programm pressed button
    code from 1 to 9. At initial button pressing role of player is selected. With second pressed button optional
    playing mode is selected depending on role. 
    For 'forward' and 'forward_old_style' role  second_pressed_button can take value 1 or value 4. With value 1
    player starts game as kick-off player,
    with value 4 player stars as non-kick-off player, which means player starts moving 10 seconds later.
    For 'run_test' role second_pressed_button can take values from 1 or value 9 with following optional modes:
    1 - 10 cycle steps walk forward
    2 - 20 cycle side step walk to right
    3 - 20 cycle side step walk to left
    4 - 20 cycle steps walk forward
    5 - 20 cycle steps with rotation to right side
    6 - 20 cycle steps with rotation to left side
    9 - 20 cycle steps of spot walk 
    All modes of run test are used with purpose to calibrate walking. After calibration is completed results of
    calibration must be input to file  Sim_params.json. Motion module is used calibration data for planning
    motions and odometry correction into localization.
    usage:
        Player(str: role, int: second_pressed_button, object: glob, object: motion, object: local)
    """
    def __init__(self, logger, role, second_pressed_button, glob, motion, local):
        self.logger = logger
        self.role = role   #'goalkeeper', 'penalty_Goalkeeper', 'forward', 'penalty_Shooter'
        self.second_pressed_button = second_pressed_button
        self.glob = glob
        self.motion = motion
        self.local = local
        self.g = None
        self.f = None

    def play_game(self):
        #success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True)
        #self.motion.pause_in_ms(200)    # this is needed for camera renewal in simulation with streaming of camera data. pause have to be same langth as camera update time
        if self.role == 'goalkeeper': self.goalkeeper_main_cycle()
        if self.role == 'goalkeeper_old_style': self.goalkeeper_old_style_main_cycle()
        if self.role == 'penalty_Goalkeeper': self.penalty_Goalkeeper_main_cycle()
        if self.role == 'forward': self.forward_main_cycle(self.second_pressed_button)
        if self.role == 'forward_old_style': self.forward_old_style_main_cycle(self.second_pressed_button)
        if self.role == 'penalty_Shooter': self.penalty_Shooter_main_cycle()
        if self.role == 'run_test': self.run_test_main_cycle(self.second_pressed_button)
        if self.role == 'rotation_test': self.rotation_test_main_cycle()
        if self.role == 'sidestep_test': self.sidestep_test_main_cycle()
        if self.role == 'dance': self.dance_main_cycle()
        if self.role == 'sprint': self.sprint_main_cycle()
        if self.role == 'marathon': self.marathon_main_cycle()


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
        self.logger.debug('self.motion.imu_body_yaw() =' + str(self.motion.imu_body_yaw()))

    def marathon_main_cycle(self):
        with open(self.glob.current_work_directory / "Init_params" / "Marathon_params.json", "r") as f:
            marathon_params = json.loads(f.read())
        proportional = marathon_params['proportional']
        differential = marathon_params['differential']
        rotation_increment = marathon_params['rotation_increment']
        direction = 0
        last_heading = 0
        stepLength = 64
        number_Of_Cycles = 500
        sideLength = 0
        self.motion.walk_Initial_Pose()
        number_Of_Cycles += 1
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            if cycle ==0 : stepLength1 = stepLength/3
            if cycle ==1 : stepLength1 = stepLength/3 * 2
            self.motion.refresh_Orientation()
            heading = self.motion.imu_body_yaw()
            rotation = direction + heading * proportional + (heading - last_heading) * differential 
            rotation = self.motion.normalize_rotation(rotation)
            #rotation = 0
            self.motion.walk_Cycle(stepLength1, sideLength, rotation, cycle, number_Of_Cycles)
            last_heading = heading
            direction += rotation_increment
        self.motion.walk_Final_Pose()

    def sprint_main_cycle(self):
        with open(self.glob.current_work_directory / "Init_params" / "Sprint_params.json", "r") as f:
            sprint_params = json.loads(f.read())
        proportional = sprint_params['proportional']
        differential = sprint_params['differential']
        last_heading = 0
        stepLength = 64
        number_Of_Cycles = 100
        sideLength = 0
        self.motion.walk_Initial_Pose()
        number_Of_Cycles += 1
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            if cycle ==0 : stepLength1 = stepLength/3
            if cycle ==1 : stepLength1 = stepLength/3 * 2
            self.motion.refresh_Orientation()
            heading = self.motion.imu_body_yaw()
            rotation = 0 + heading * proportional + (heading - last_heading) * differential 
            rotation = self.motion.normalize_rotation(rotation)
            #rotation = 0
            self.motion.walk_Cycle(stepLength1, sideLength, rotation, cycle, number_Of_Cycles)
            last_heading = heading
        self.motion.walk_Final_Pose()

    def run_test_main_cycle(self, pressed_button):
        """
        For 'run_test' role second_pressed_button can take values from 1 or value 9 with following optional modes:
        1 - 10 cycle steps walk forward
        2 - 20 cycle side step walk to right
        3 - 20 cycle side step walk to left
        4 - 20 cycle steps walk forward
        5 - 20 cycle steps with rotation to right side
        6 - 20 cycle steps with rotation to left side
        9 - 20 cycle steps of spot walk 
        All modes of run test are used with purpose to calibrate walking. After calibration is completed results of
        calibration must be input to file  Sim_params.json. Motion module is used calibration data for planning
        motions and odometry correction into localization.
        usage:
            self.run_test_main_cycle(int: pressed_button)
        """
        if pressed_button == 2 or pressed_button ==3 :
            self.sidestep_test_main_cycle(pressed_button)
            return
        if pressed_button == 5 or pressed_button ==6 :
            self.rotation_test_main_cycle(pressed_button)
            return
        stepLength = 64
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
        stepLength = 0
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
        """
        This module normalizes yaw according to internal rule: -pi <= yaw <= pi
        usage:
            float: yaw = self.norm_yaw(float: yaw)
            yaw - orientation on horizontal surface in radians, 
                  zero value orientation is directed along X axis
        """
        yaw %= 2 * math.pi
        if yaw > math.pi:  yaw -= 2* math.pi
        if yaw < -math.pi: yaw += 2* math.pi
        return yaw

    def forward_main_cycle(self, pressed_button):
        """
        Main cycle method for 'forward' role of player.
        usage:
            self.forward_main_cycle(int: pressed_button)
        """
        second_player_timer = self.motion.game_time()                       # ignition of timer for non-kick-off forward player
        self.f = Forward_Vector_Matrix(self.logger, self.motion, self.local, self.glob)
        while (True):
            if self.motion.falling_Flag != 0:                               # falling_Flag variable is used to with purpose to deliver
                                                                            # to strategy falling event or request to stop cycle. 
                                                                            # falling_Flag = 0 if nothing happened, = 1 if there was falling to stomach
                                                                            # =-1 if there was falling to back, = 2 if there was falling to left, 
                                                                            # = -2 if there was falling to right, = 3 if there was request to stop cycle.  
                if self.motion.falling_Flag == 3:
                    self.motion.play_Soft_Motion_Slot(name = 'Initial_Pose')
                    break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()                          # after falling localization must be blurered
            success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True)
            self.logger.debug( 'napravl =' + str(napravl) + ' dist = ' + str(dist) )
            #if self.glob.SIMULATION == 2 and self.glob.wifi_params['WIFI_IS_ON']: self.local.report_to_WIFI()  # telemetry for real robot 
            if pressed_button == 4 and (self.motion.game_time() - second_player_timer) < 10 : 
                continue                                                    # motion is ignored diring 10 seconds for non-kick-off player 
            self.f.dir_To_Guest()                                           # loading kick direction from strategy_data.json into self.f.direction_To_Guest
            self.logger.debug('direction_To_Guest = ' + str(math.degrees(self.f.direction_To_Guest)) + 'degrees')
            self.logger.debug('coord =' + str(self.glob.pf_coord) + ' ball =' + str(self.glob.ball_coord))
            if dist == 0 and success_Code == False:                         # condition when robot doesn't find ball 
                self.motion.turn_To_Course(self.glob.pf_coord[2]+ 2 * math.pi / 3)  # robot turns to 1/3 of round from current orientation
                continue
            # following 4 operators detect is the player in fast kick position or is the player in front of ball
            # if player is in fast kick position then it proceed to near_distance_ball_approach_and_kick mandatory.
            # if player is in front of ball then it proceed to far_distance_plan_approach mandatory
            # if player isn't in fast kick position or isn't in front of ball then selection between near_distance_ball_approach_and_kick
            # or far_distance_plan_approach depends on distance to ball
            player_from_ball_yaw = self.motion.p.coord2yaw(self.glob.pf_coord[0] - self.glob.ball_coord[0],
                                                          self.glob.pf_coord[1] - self.glob.ball_coord[1]) - self.f.direction_To_Guest
            player_from_ball_yaw = self.norm_yaw(player_from_ball_yaw)
            player_in_front_of_ball = -math.pi/2 < player_from_ball_yaw < math.pi/2
            player_in_fast_kick_position = (player_from_ball_yaw > 2 or player_from_ball_yaw < -2) and dist < 0.6 
            if (dist > 0.35  or player_in_front_of_ball) and not player_in_fast_kick_position:
                if dist > 1: stop_Over = True
                else: stop_Over = False
                self.motion.far_distance_plan_approach(self.glob.ball_coord, self.f.direction_To_Guest, stop_Over = stop_Over)
                continue
            self.motion.turn_To_Course(self.f.direction_To_Guest)
            small_kick = False
            if self.f.kick_Power > 1: small_kick = True
            success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False, small_kick = small_kick)
        

    def forward_old_style_main_cycle(self, pressed_button):
        """
        Main cycle method for 'forward_old_style' role of player.
        usage:
            self.forward_main_cycle(int: pressed_button)
        """
        
        second_player_timer = self.motion.game_time()                       # ignition of timer for non-kick-off forward player
        self.f = Forward(self.logger, self.motion, self.local, self.glob)
        while (True):
            if self.motion.falling_Flag != 0:                               # falling_Flag variable is used to with purpose to deliver
                                                                            # to strategy falling event or request to stop cycle. 
                                                                            # falling_Flag = 0 if nothing happened, = 1 if there was falling to stomach
                                                                            # =-1 if there was falling to back, = 2 if there was falling to left, 
                                                                            # = -2 if there was falling to right, = 3 if there was request to stop cycle.
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()                          # after falling localization must be blurered
            success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True)
            self.logger.debug( 'napravl =' + str(napravl) + 'dist = ' + str(dist) )
            if pressed_button == 4 and (self.motion.game_time() - second_player_timer) < 10 : 
                continue                                                    # motion is ignored diring 10 seconds for non-kick-off player
            if dist == 0 and success_Code == False:                         # condition when robot doesn't find ball
                self.motion.turn_To_Course(self.glob.pf_coord[2]+ 2 * math.pi / 3)  # robot turns to 1/3 of round from current orientation
                continue
            if dist > 0.5  or self.glob.pf_coord[0] > self.glob.ball_coord[0] :
                self.motion.far_distance_ball_approach(self.glob.ball_coord)
                self.f.turn_Face_To_Guest()
                continue
            success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False)


    def goalkeeper_main_cycle(self):        
        """
        goalkeeper main cycle method is based on vector matrix strategy. Goalkeeper doesn't leave goals too far. 
        Supposed that goalkeeper starts game at point on middle of goal line.
        After 10 seconds from game start goalkeeper moves to duty position which depends on detected ball position.
        In case if ball appears in dangetous position goalkeeper attcks ball. 
        """
        def ball_position_is_dangerous(row, col):
            """
            Method detects if position of ball is risky.
            Risky position is qualified if ball is in nearest third of field to own goals according to X coordinate.
            From risky positions of ball positions in corners are excluded 
            usage: 
                bool: danger = ball_position_is_dangerous(int: row, int: col)
                row - row of field sector according to matrix
                col - column of field sector according to matrix
            """
            danger = False
            danger = (col <= (round(self.glob.COLUMNS / 3) - 1))
            if ((row <= (round(self.glob.ROWS / 3) - 1) or row >= round(self.glob.ROWS * 2 / 3)) and col == 0) or (col == 1 and (row == 0 or row == (self.glob.ROWS -1))):
               danger = False
            return danger
        second_player_timer = self.motion.game_time()
        self.f = Forward_Vector_Matrix(self.logger, self.motion, self.local, self.glob)
        #self.motion.near_distance_omni_motion(400, 0)                    # get out from goal
        fast_Reaction_On = True
        while (True):
            if self.motion.falling_Flag != 0:                               # falling_Flag variable is used to with purpose to deliver
                                                                            # to strategy falling event or request to stop cycle. 
                                                                            # falling_Flag = 0 if nothing happened, = 1 if there was falling to stomach
                                                                            # =-1 if there was falling to back, = 2 if there was falling to left, 
                                                                            # = -2 if there was falling to right, = 3 if there was request to stop cycle.
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()
            if self.glob.ball_coord[0] <= 0.15:
                success_Code, napravl, dist, speed =  self.motion.watch_Ball_In_Pose()                  # looks with increased attention
            else: 
                success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = fast_Reaction_On)
            if abs(speed[0]) > 0.02 and dist < 1 :                         # if dangerous tangential speed
                fast_Reaction_On = True
                if speed[0] > 0:
                    if self.glob.pf_coord[1] < 0.35:
                        self.motion.play_Soft_Motion_Slot(name ='PenaltyDefenceL')      # falling to left side
                else:
                    if self.glob.pf_coord[1] > -0.35:
                        self.motion.play_Soft_Motion_Slot(name ='PenaltyDefenceR')      # falling to right side
                self.motion.pause_in_ms(3000)
                continue
            if speed[1] < - 0.01 and dist < 1.5 :                          # if dangerous front speed
                fast_Reaction_On = True
                self.motion.play_Soft_Motion_Slot(name = 'PanaltyDefenceReady_Fast')    # preparing for splits
                self.motion.play_Soft_Motion_Slot(name = 'PenaltyDefenceF')             # sit to splits
                self.motion.pause_in_ms(3000)
                self.motion.play_Soft_Motion_Slot(name = 'Get_Up_From_Defence')
                continue
            if (self.motion.game_time() - second_player_timer) < 10 : continue
            row, col = self.f.dir_To_Guest()
            if dist == 0 and success_Code == False:                             # if ball was not detected at in viewable sectors
                self.logger.info('goalkeeper turn_To_Course(pi*2/3)')
                self.motion.turn_To_Course(self.glob.pf_coord[2]+ 2 * math.pi / 3)
                continue
            if ball_position_is_dangerous(row, col):
                # attacking of ball
                fast_Reaction_On = True
                player_from_ball_yaw = self.motion.p.coord2yaw(self.glob.pf_coord[0] - self.glob.ball_coord[0], self.glob.pf_coord[1] - self.glob.ball_coord[1]) - self.f.direction_To_Guest
                player_from_ball_yaw = self.norm_yaw(player_from_ball_yaw)
                player_in_front_of_ball = -math.pi/2 < player_from_ball_yaw < math.pi/2
                player_in_fast_kick_position = (player_from_ball_yaw > 2 or player_from_ball_yaw < -2) and dist < 0.6 
                if (dist > 0.35  or player_in_front_of_ball) and not player_in_fast_kick_position:
                    if dist > 1: stop_Over = True
                    else: stop_Over = False
                    self.logger.info('goalkeeper far_distance_plan_approach')
                    self.motion.far_distance_plan_approach(self.glob.ball_coord, self.f.direction_To_Guest, stop_Over = stop_Over)
                    #self.f.turn_Face_To_Guest()
                    continue
                self.logger.info('goalkeeper turn_To_Course(direction_To_Guest)')
                self.motion.turn_To_Course(self.f.direction_To_Guest)
                small_kick = False
                if self.f.kick_Power > 1: small_kick = True
                self.logger.info('goalkeeper near_distance_ball_approach_and_kick')
                success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False, small_kick = small_kick)

            else:
                # returning to duty position
                fast_Reaction_On = False
                duty_x_position =  min((-self.glob.landmarks['FIELD_LENGTH']/2 + 0.4),(self.glob.ball_coord[0]-self.glob.landmarks['FIELD_LENGTH']/2)/2)
                duty_y_position = self.glob.ball_coord[1] * (duty_x_position + self.glob.landmarks['FIELD_LENGTH']/2) / (self.glob.ball_coord[0] + self.glob.landmarks['FIELD_LENGTH']/2) 
                duty_distance = math.sqrt((duty_x_position - self.glob.pf_coord[0])**2 + (duty_y_position - self.glob.pf_coord[1])**2)
                self.logger.debug('duty_x_position =' + str(duty_x_position) + ' duty_y_position =' + str(duty_y_position))
                if duty_distance < 0.2 : continue
                elif duty_distance <  3: #   0.6 :
                    self.logger.info('goalkeeper turn_To_Course(0)')
                    self.motion.turn_To_Course(0)
                    duty_direction = self.motion.p.coord2yaw(duty_x_position - self.glob.pf_coord[0], duty_y_position - self.glob.pf_coord[1])  
                    self.logger.info('goalkeeper near_distance_omni_motion')
                    self.motion.near_distance_omni_motion(duty_distance * 1000, duty_direction)
                    self.logger.info('goalkeeper turn_To_Course(0)')
                    self.motion.turn_To_Course(0)
                else:
                    self.motion.far_distance_plan_approach([duty_x_position + 0.25, duty_y_position], 0, stop_Over = False)
        self.motion.play_Soft_Motion_Slot(name = 'Initial_Pose')

    def goalkeeper_old_style_main_cycle(self):
        """
        main cycle for old style goalkeeper strategy
        """
        self.motion.near_distance_omni_motion(200, 0)                    # get out from goal line
        self.g = GoalKeeper(self.logger, self.motion, self.local, self.glob)
        while (True):
            dist = -1.0
            if self.motion.falling_Flag != 0:                               # falling_Flag variable is used to with purpose to deliver
                                                                            # to strategy falling event or request to stop cycle. 
                                                                            # falling_Flag = 0 if nothing happened, = 1 if there was falling to stomach
                                                                            # =-1 if there was falling to back, = 2 if there was falling to left, 
                                                                            # = -2 if there was falling to right, = 3 if there was request to stop cycle.
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()
                self.g.turn_Face_To_Guest()
            while(dist < 0):
                a, dist,napravl, speed = self.g.find_Ball()
                self.logger.debug('speed = ' + str(speed) + ' dist  =' + str(dist) + ' napravl =' + str(napravl))
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
        """
        main cycle for penalty striker 
        """
        self.f = Forward_Vector_Matrix(self.logger, self.motion, self.local, self.glob)
        while (True):
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()
            success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True)
            self.f.dir_To_Guest()
            self.logger.debug('ball_coord = ' + str(self.glob.ball_coord))
            self.logger.debug('direction_To_Guest = ' + str(math.degrees(self.f.direction_To_Guest)) + 'degrees')
            if dist == 0 and success_Code == False:
                self.motion.turn_To_Course(self.glob.pf_coord[2]+ 2 * math.pi / 3)
                continue
            if dist > 0.35 or self.glob.pf_coord[0] - 0.2 > self.glob.ball_coord[0]:
                if dist > 1: stop_Over = True
                else: stop_Over = False
                self.motion.far_distance_plan_approach(self.glob.ball_coord, self.f.direction_To_Guest, stop_Over = stop_Over)
            kick_direction = self.f.direction_To_Guest
            self.motion.turn_To_Course(kick_direction)
            small_kick = False
            if self.f.kick_Power > 1: small_kick = True
            success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False, small_kick = small_kick)
            #success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False)

    def penalty_Goalkeeper_main_cycle(self):
        self.g = GoalKeeper(self.logger, self.motion, self.local, self.glob)
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
                self.logger.debug('speed = ' + str(speed) + 'dist  =' + str(dist) + 'napravl =' + str(napravl))
                if abs(speed[0]) > 0.004 and dist < 1 :                         # if dangerous tangential speed
                    if speed[0] > 0:
                        self.motion.play_Soft_Motion_Slot(name ='PenaltyDefenceL')
                        self.motion.pause_in_ms(2000)
                        self.motion.play_Soft_Motion_Slot(name = 'Get_Up_Left')
                    else:
                        self.motion.play_Soft_Motion_Slot(name ='PenaltyDefenceR')
                        self.motion.pause_in_ms(2000)
                        self.motion.play_Soft_Motion_Slot(name = 'Get_Up_Right')
                    
                    continue
                if speed[1] < - 0.03 and dist < 1.5 :                          # if dangerous front speed
                    self.motion.play_Soft_Motion_Slot(name = 'PanaltyDefenceReady_Fast')
                    self.motion.play_Soft_Motion_Slot(name = 'PenaltyDefenceF')
                    self.motion.pause_in_ms(2000)
                    self.motion.play_Soft_Motion_Slot(name = 'Get_Up_From_Defence')

                if (dist == 0 and napravl == 0) or dist > 2.5:
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
            #for i in range(10):
            #    self.motion.play_Soft_Motion_Slot( name = 'Dance_6_1')
            self.motion.play_Soft_Motion_Slot( name = 'Dance_7')
            self.motion.play_Soft_Motion_Slot( name = 'Dance_2')
            self.motion.play_Soft_Motion_Slot( name = 'Dance_4')


if __name__=="__main__":
    pass