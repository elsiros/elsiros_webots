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

    def coordinate_record(self, odometry = False, shift = False):
        x,y,yaw = self.motion.sim_Get_Robot_Position()
        self.glob.pf_coord = [x * self.side_factor, y * self.side_factor, yaw]
        #self.glob.pf_coord = [x * self.side_factor, y * self.side_factor, yaw + math.pi * (1 - self.side_factor)/2]
        self.glob.pf_coord[2] = self.normalize_yaw(self.glob.pf_coord[2])
        #self.glob.pf_coord = self.call_Par_Filter.return_coord()
        if odometry:
            if shift:
                self.glob.pf_coord[0] += self.coord_shift[0]
                self.glob.pf_coord[1] += self.coord_shift[1]
                self.glob.pf_coord[2] += self.coord_shift[2]
            self.robot_moved = True
        else:
            x,y,yaw = self.motion.sim_Get_Robot_Position()
            self.glob.pf_coord = [x * self.side_factor, y * self.side_factor, yaw]
            #self.glob.pf_coord = [x * self.side_factor, y * self.side_factor, yaw + math.pi * (1 - self.side_factor)/2]
            self.glob.pf_coord[2] = self.normalize_yaw(self.glob.pf_coord[2])
        if self.glob.wifi_params['WIFI_IS_ON']: self.report_to_WIFI()
        if (self.glob.SIMULATION == 1 or self.glob.SIMULATION == 0 or self.glob.SIMULATION == 3):
            timer1 = time.perf_counter() - self.timer0
            Dummy_PF_position = [self.glob.pf_coord[0] * self.side_factor,
                                self.glob.pf_coord[1] * self.side_factor, 0.01]
            Dummy_PF_orientation = [0, 0, self.glob.pf_coord[2] + (math.pi/2 *(1 - self.side_factor))]
            returnCode, Dummy_1position= self.motion.sim.simxGetObjectPosition(
                                         self.motion.clientID, self.motion.Dummy_1Handle ,
                                         -1, self.motion.sim.simx_opmode_streaming)
            Dummy_PF_H = []
            yaw_rad = self.motion.body_euler_angle['yaw']
            if LOCALISATION_VISUALISATION_IS_ON :
                self.visualisation.localisation_points(self.motion, Dummy_PF_position, Dummy_PF_orientation, Dummy_PF_H)
            if OBSTACLE_VISUALISATION_IS_ON and self.glob.obstacleAvoidanceIsOn and odometry == False:
                scene_obstacles = []
                for i in range(len(self.glob.obstacles)):
                    obstacle_for_scene = []
                    obstacle_for_scene.append((self.glob.obstacles[i][0] + Dummy_1position[0]) * self.side_factor - self.glob.pf_coord[0])
                    obstacle_for_scene.append((self.glob.obstacles[i][1] + Dummy_1position[1]) * self.side_factor - self.glob.pf_coord[1])
                    obstacle_for_scene.append(self.glob.obstacles[i][2])
                    scene_obstacles.append(obstacle_for_scene)
                self.visualisation.obstacle_mark(self.motion, scene_obstacles)

    def report_to_WIFI(self):
        message_to_Host = {"ID": 0, "x": 0.0, "y": 0.0, "yaw": 0.0, "bx": 0.0, "by": 0.0, "bytes": 0}
        if self.glob.SIMULATION == 2 : message_to_Host["ID"] = self.glob.wifi_params['ROBOT_ID']
        else: message_to_Host["ID"] = str(self.motion.robot_Number)
        message_to_Host["x"] = round(self.glob.pf_coord[0], 3)
        message_to_Host["y"] = round(self.glob.pf_coord[1], 3)
        message_to_Host["yaw"] = round(self.glob.pf_coord[2], 3)
        message_to_Host["bx"] = round(self.glob.ball_coord[0], 3)
        message_to_Host["by"] = round(self.glob.ball_coord[1], 3)
        message_to_Host["bytes"] = len(bytes(array.array('I', (i for i in range(1)))))
        data = str(message_to_Host).encode()
        try:
            self.glob.udp_socket.sendto(data, self.glob.target_wifi_address)
            #self.glob.udp_socket.sendto('particles'.encode(), self.glob.target_wifi_address)
            if str(self.motion.robot_Number) == '':
                self.glob.udp_socket.sendto(self.glob.pf_alloc1, self.glob.target_wifi_address)
        except Exception: 
            #print('failed to send')
            pass

    def localisation_Complete(self):
        x,y,yaw = self.motion.sim_Get_Robot_Position()
        self.glob.pf_coord = [x * self.side_factor, y * self.side_factor, yaw]
        #self.glob.pf_coord = [x * self.side_factor, y * self.side_factor, yaw + math.pi * (1 - self.side_factor)/2]
        self.glob.pf_coord[2] = self.normalize_yaw(self.glob.pf_coord[2])
        if self.glob.obstacleAvoidanceIsOn: self.group_obstacles()
        self.coordinate_record()
        return True

    def group_obstacles(self):
        grouped_obstacles = []
        #uprint('obstacles(raw): ', self.glob.obstacles)
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

    def read_Localization_marks(self, img):
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
            self.detect_obstacles(img)


if __name__=="__main__":
    print('This is not main module!')









