"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of A. Babaev.

This module is assisting localization

"""



import sys, os
import math, time, json, array

LOCALISATION_VISUALISATION_IS_ON = False
OBSTACLE_VISUALISATION_IS_ON = False


class Local():
    def __init__ (self, motion, glob, coord_odometry = [0.0,0.0,0.0]):
        self.motion = motion
        self.glob = glob
        self.coord_shift = [0.0, 0.0, 0.0]
        self.timer0 = time.perf_counter()
        if abs(motion.direction_To_Attack) < 1: self.side_factor = 1
        else: self.side_factor = -1
        from .class_Visualisation import Visualisation
        self.visualisation = Visualisation()
        self.robot_moved = False

    def coordinate_fall_reset(self):
        #self.call_Par_Filter.pf.fall_reset()
        pass

    def coordinate_trust_estimation(self):
        #return self.call_Par_Filter.pf.consistency
        return 1

    def normalize_yaw(self, yaw):
        if abs(yaw) > 2 * math.pi: yaw %= (2 * math.pi)
        if yaw > math.pi : yaw -= (2 * math.pi)
        if yaw < -math.pi : yaw += (2 * math.pi)
        return yaw


    def correct_yaw_in_pf(self):
        x,y,yaw = self.motion.sim_Get_Robot_Position()
        #self.glob.pf_coord[2] = yaw + math.pi * (1 - self.side_factor)/2
        self.glob.pf_coord[2] = self.normalize_yaw(self.glob.pf_coord[2])

    def coordinate_record(self):
        x,y,yaw = self.motion.sim_Get_Robot_Position()
        self.glob.pf_coord = [x * self.side_factor, y * self.side_factor, yaw]
        #self.glob.pf_coord = [x * self.side_factor, y * self.side_factor, yaw + math.pi * (1 - self.side_factor)/2]
        self.glob.pf_coord[2] = self.normalize_yaw(self.glob.pf_coord[2])


    def localisation_Complete(self):
        x,y,yaw = self.motion.sim_Get_Robot_Position()
        self.glob.pf_coord = [x * self.side_factor, y * self.side_factor, yaw]
        #self.glob.pf_coord = [x * self.side_factor, y * self.side_factor, yaw + math.pi * (1 - self.side_factor)/2]
        self.glob.pf_coord[2] = self.normalize_yaw(self.glob.pf_coord[2])
        if self.glob.obstacleAvoidanceIsOn: self.group_obstacles()
        return True

    def group_obstacles(self):
        grouped_obstacles = []
        print('obstacles(raw): ', self.glob.obstacles)
        while(len(self.glob.obstacles) > 0):
            obstacle0 = self.glob.obstacles.pop(0)
            group_number = 1
            k = 0
            for i in range(len(self.glob.obstacles)):
                united_obstacles = math.sqrt((obstacle0[0]-self.glob.obstacles[i-k][0])**2 + (obstacle0[1]-self.glob.obstacles[i-k][1])**2)\
                                               < (obstacle0[2] + self.glob.obstacles[i-k][2])/2
                if united_obstacles:
                    group_number += 1
                    new_size = math.sqrt((obstacle0[0]-self.glob.obstacles[i-k][0])**2 + (obstacle0[1]-self.glob.obstacles[i-k][1])**2)\
                                               + (obstacle0[2] + self.glob.obstacles[i-k][2])/2
                    obstacle0 = [(obstacle0[0]*(group_number-1) + self.glob.obstacles[i-k][0])/group_number,
                                 (obstacle0[1]*(group_number-1) + self.glob.obstacles[i-k][1])/group_number,
                                 (obstacle0[2]*(group_number-1) + new_size)/group_number,]
                    self.glob.obstacles.pop(i-k)
                    k += 1
            if obstacle0[2] > 0.1:
                grouped_obstacles.append(obstacle0)
        self.glob.obstacles = []
        for obstacle in grouped_obstacles:
            global_x = self.glob.pf_coord[0] + obstacle[0] * math.cos(self.glob.pf_coord[2]) - obstacle[1] * math.sin(self.glob.pf_coord[2])
            global_y = self.glob.pf_coord[1] + obstacle[1] * math.cos(self.glob.pf_coord[2]) + obstacle[0] * math.sin(self.glob.pf_coord[2])
            if abs(global_y) <= self.glob.landmarks['FIELD_WIDTH']/2 and abs(global_x) <= self.glob.landmarks['FIELD_LENGTH']/2:
                obstacle[0] = global_x
                obstacle[1] = global_y
                self.glob.obstacles.append(obstacle)
        print('obstacles(processed): ', self.glob.obstacles)

    def read_Localization_marks(self):
        #self.quality = 0
        #post_list1 = self.detect_Post_In_image(img, "blue posts")
        #for post in post_list1:
        #    self.post_data_in_pose[self.post_data_in_pose_number][0] = int(post[0] * 2000)
        #    self.post_data_in_pose[self.post_data_in_pose_number][1] = int(post[1] * 2000)
        #    self.post_data_in_pose_number += 1
        ##if len(post_list1) != 0:
        ##    self.post_data_in_pose.extend(post_list1)
        #post_list2 = self.detect_Post_In_image(img, "yellow posts")
        #for post in post_list2:
        #    self.post_data_in_pose[self.post_data_in_pose_number][0] = int(post[0] * 2000)
        #    self.post_data_in_pose[self.post_data_in_pose_number][1] = int(post[1] * 2000)
        #    self.post_data_in_pose_number += 1
        ##if len(post_list2) != 0:
        ##    self.post_data_in_pose.extend(post_list2)
        #if self.USE_LANDMARKS_FOR_LOCALISATION == True:
        #    if self.USE_PENALTY_MARKS_FOR_LOCALISATION ==True: self.detect_penalty_marks(img)
        #    if self.USE_LINES_FOR_LOCALISATION == True : self.detect_line_in_image(img)
        if self.robot_moved == True:
            self.robot_moved = False
            self.glob.obstacles.clear()
        if self.glob.obstacleAvoidanceIsOn:
            self.motion.sim_Get_Obstacles()


if __name__=="__main__":
    print('This is not main module!')









