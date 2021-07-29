

import sys, os
import math, time, json, array

LOCALISATION_VISUALISATION_IS_ON = True
OBSTACLE_VISUALISATION_IS_ON = False


class Local():
    def __init__ (self, motion, glob, coord_odometry = [0.0,0.0,0.0]):
        self.motion = motion
        self.glob = glob
        self.coord_shift = [0.0, 0.0, 0.0]
        self.timer0 = time.perf_counter()
        if abs(motion.direction_To_Attack) < 1: self.side_factor = 1
        else: self.side_factor = -1
        from class_Visualisation import Visualisation
        self.visualisation = Visualisation()

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
        self.glob.pf_coord[2] = yaw + math.pi * (1 - self.side_factor)/2

    def coordinate_record(self, odometry = False, shift = False):
        x,y,yaw = self.motion.sim_Get_Robot_Position()
        self.glob.pf_coord = [x * self.side_factor, y * self.side_factor, yaw + math.pi * (1 - self.side_factor)/2]
        #self.glob.pf_coord = self.call_Par_Filter.return_coord()
        if odometry:
            if shift:
                self.glob.pf_coord[0] += self.coord_shift[0]
                self.glob.pf_coord[1] += self.coord_shift[1]
                self.glob.pf_coord[2] += self.coord_shift[2]
        else:
            x,y,yaw = self.motion.sim_Get_Robot_Position()
            self.glob.pf_coord = [x * self.side_factor, y * self.side_factor, yaw + math.pi * (1 - self.side_factor)/2]
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
        self.glob.pf_coord = [x * self.side_factor, y * self.side_factor, yaw + math.pi * (1 - self.side_factor)/2]
        if self.glob.obstacleAvoidanceIsOn: self.group_obstacles()
        self.coordinate_record()
        return True


if __name__=="__main__":
    print('This is not main module!')









