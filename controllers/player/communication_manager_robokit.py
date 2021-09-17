"""[summary]
Returns:
[type]: [description]
"""
import queue
import time
import logging
from threading import Thread, Lock
import json
import math


from robot_client import RobotClient

class Blurrer():    
    def __init__(self):
        params = self.load_json("blurrer.json")
        self.object_angle_noize = params["object_angle_noize"]
        self.object_distance_noize = params["object_distance_noize"]
        self.observation_bonus = params["observation_bonus"]
        self.step_loss = params["step_loss"]
        self.constant_loc_noize = params["constant_loc_noize"]
    
    def load_json(self, filename):
        with open(filename) as f:
            params = json.load(f)
        return params

class Model():
    def __init__(self, robot):
        self.robot = robot
        self.max_visible_area = 4
        self.min_visible_area = 0
        self.fov_x = math.radians(45) / 2
        self.fov_y = math.radians(60) / 2
        self.robot_height = 0.413

    def check_robot_stand(self):
        servos_sum = 0
        for angle in self.robot.last_message.values():
            servos_sum += angle
        
        # print(f"Sum angle: {servos_sum}")
        if servos_sum >= 0.01:
            return False
        else:
            return True

    def check_object_in_frame(self, distance, course):
        right_yaw_visible_area = -self.robot.last_head_yaw - self.fov_y
        left_yaw_visible_area = -self.robot.last_head_yaw + self.fov_y

        if -self.robot.last_head_pitch < self.fov_x:
            top_distance_visible_area = self.max_visible_area
        else:
            top_distance_visible_area = self.robot_height * \
                                        math.tan(math.pi/2 + \
                                        self.robot.last_head_pitch + \
                                        self.fov_x)

        if -self.robot.last_head_pitch > math.pi/2 - self.fov_x:
            bottom_distance_visible_area = self.min_visible_area
        else:
            bottom_distance_visible_area = self.robot_height * \
                                        math.tan(math.pi/2 + \
                                        self.robot.last_head_pitch - \
                                        self.fov_x)

        # print(f"right_yaw_visible_area: {right_yaw_visible_area}, left_yaw_visible_area: {left_yaw_visible_area}, \
        #         top_distance_visible_area: {top_distance_visible_area}, bottom_distance_visible_area: {bottom_distance_visible_area} \
        #         distance: {distance}, course: {course}, self.fov_y: {self.fov_y}, self.fov_x: {self.fov_x},\
        #         self.robot_yaw: {self.robot.last_head_yaw} self.robot_pitch: {self.robot.last_head_pitch}")
        if ((bottom_distance_visible_area < distance < top_distance_visible_area) and 
            (right_yaw_visible_area < course < left_yaw_visible_area)):
            return True
        else:
            return False

    @staticmethod
    def dist(p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        
    def get_distance_course(self, x, y):
        robot_gps = self.robot.get_sensor("gps_body")
        if not robot_gps:
            return []

        robot_pos = robot_gps["position"]
        distance = self.dist(robot_pos, (x, y))

        robot_imu = self.robot.get_sensor("imu_body")

        if not robot_imu:
            return []

        robot_orientation = robot_imu["position"]

        angle = math.atan2(robot_pos[1] - y, -(robot_pos[0] - x)) - robot_orientation[2]
        # if (x < robot_pos[0]):
        #     angle = math.pi - angle
        # angle = angle * (robot_pos[1] - y)/abs(robot_pos[1] - y) - robot_orientation[2]

        return (distance, angle)

    def proccess_data(self, x, y):
        if not self.check_robot_stand():
            # print("WARNING: Robot in not standing")
            return []
        res = self.get_distance_course(x, y)
        if not res:
            # print("WARNING: Imu or gps not available")
            return []
        distance, angle = res
        if self.check_object_in_frame(distance, angle):
            return [-angle, distance]
        else:
            # print("WARNING: Ball is not in the frame")
            return []

class CommunicationManager():
    """[summary]
    """
    def __init__(self, maxsize=1, host='127.0.0.1', port=10001, team_color="RED", player_number = 1, time_step = 15):
        # logging.basicConfig(filename=f'cm_robokit{port}.txt', encoding="utf-8", level=logging.DEBUG)
        verbosity = 4
        self.client = RobotClient(host, port, verbosity)
        self.client.connect_client()
        self.maxsize = maxsize
        self.sensors = {}
        self.robot_color = team_color
        self.robot_number = player_number
        self.time_step = time_step
        self.tx_mutex = Lock()
        self.tx_message = {}
        self.last_message = {}
        self.last_head_yaw = 0
        self.last_head_pitch = 0

        self.model = Model(self)
        self.current_time = 0

        self.sensor_time_step = time_step * 4
        #self.sensor_time_step = 15

        sensors = {"imu_body": self.time_step, "recognition": self.sensor_time_step, "gps_body": self.sensor_time_step}
        # sensors = {"gps_body": 5, "imu_head": 5, "imu_body": 5,  "camera": 20}#
        self.enable_sensors(sensors)
        self.thread = Thread(target = self.run)
        self.thread.start()
        

    def enable_sensors(self, sensors) -> None:
        for sensor in sensors:
            self.client.initial(sensor, sensors[sensor])
            if sensor == "recognition":
                self.sensors.update({"BALL": {} })
                self.sensors.update({"RED_PLAYER_1": {} })
                self.sensors.update({"RED_PLAYER_2": {} })
                self.sensors.update({"BLUE_PLAYER_1": {} })
                self.sensors.update({"BLUE_PLAYER_2": {} })

            self.sensors.update({str(sensor): {}})
        self.sensors.update({"time": {}})
        self.client.send_request("init")

    def get_sensor(self, name) -> dict:
        return self.sensors[name]

    def send_message(self):
        self.tx_mutex.acquire()
        if self.tx_message:
            self.client.send_request("positions", self.tx_message)
            self.tx_message = {}
        self.tx_mutex.release()

    def update_history(self, message):
        for sensor in message:
            if (sensor == "time"):
                delta = message[sensor]['sim time'] - self.current_time
                if delta > 5:
                    pass
                    #print(f"WARNING! Large protobuf time rx delta = {delta}")                
                self.current_time = message[sensor]['sim time']
                #logging.debug("Getting servo commands:")
                #logging.debug(f"New simulation time: {self.current_time}")
                #logging.debug(data)                
            self.sensors[sensor] = message[sensor]

    def time_sleep(self, t = 0)->None:
        print(f"Emulating delay of {t*1000} ms")
        start_time = self.current_time
        while (self.current_time - start_time < t * 1000):
            time.sleep(0.001)

    def get_imu_body(self):
        # self.last_imu_body = self.get_sensor("imu_body")
        #self.time_sleep()
        return self.get_sensor("imu_body")

    def get_imu_head(self):
        #self.time_sleep()
        return self.get_sensor("imu_head")

    def get_localization(self):
        self.time_sleep(0.5)
        return self.get_sensor("gps_body")

    def get_ball(self):
        # self.time_sleep(0.1)
        ball = self.get_sensor("BALL").copy()
        # print("Abs ball: ", ball)
        if ball:
            ball_pos = ball["position"]
            if not ball_pos:
                return []
            updated_ball_pos = self.model.proccess_data(ball_pos[0], ball_pos[1])
            ball["position"] = updated_ball_pos
            return ball
        else:
            return {}

    def get_opponents(self):
        self.time_sleep(0.1)
        color = "BLUE" if self.robot_color == "RED" else "RED"    
        return [self.get_sensor(color+"_PLAYER_1"), self.get_sensor(color+"_PLAYER_2")]

    def get_teammates(self):
        self.time_sleep(0.1)
        number = 1 if self.robot_number == 2 else 1
        return self.get_sensor(f"{self.robot_color}_PLAYER_+{number}")
    
    def get_time(self):
        return self.get_sensor("time")

    def send_servos(self, data = {}):
        #self.time_sleep(0)
        #logging.debug("Getting servo commands:")
        #logging.debug(f"Time: {self.current_time}")
        #logging.debug(data)

        self.tx_mutex.acquire()
        self.tx_message = data
        self.tx_mutex.release()
        # logging.debug("Getting servo commands:")
        # logging.debug(f"Time: {self.current_time}")
        # logging.debug(data)
        if "right_hip_yaw" in data.keys():
            self.last_message = data

        if "head_yaw" in data.keys():
            self.last_head_yaw = data["head_yaw"]
        if "head_pitch" in data.keys():
            self.last_head_pitch = data["head_pitch"]
        #self.add_to_queue((data, {}))
        #return 0 

    def run(self):
        while(True):
            do_not_block = True
            if do_not_block:
                # Sending/receiving protobuf in non-blocking way 
                # If we have any data to send - do sending
                # If full packet data is ready in socket - receive it, otherwise switch to check if sending is needed
                self.send_message()
                messages_list = self.client.receive2()
                for message in messages_list:
                    self.update_history(message)
            else:
                # Sending/receiving protobuf in blocking way:
                # wait for a message ready to be sent, then send it
                # Then wait for a message to be received, and receive it
                self.send_message()
                message = []
                while not message:
                    message = self.client.receive2()
                self.update_history(message)
            
            # print("get_ball: ", self.get_ball())

    def test_run(self):
        # пример отправки данных серв
        self.WBservosList = ["right_ankle_roll", "right_ankle_pitch", "right_knee", "right_hip_pitch",
                             "right_hip_roll", "right_hip_yaw", "right_elbow_pitch", "right_shoulder_twirl",
                             "right_shoulder_roll", "right_shoulder_pitch", "pelvis_yaw", "left_ankle_roll",
                             "left_ankle_pitch", "left_knee", "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
                             "left_elbow_pitch", "left_shoulder_twirl", "left_shoulder_roll",
                             "left_shoulder_pitch", "head_yaw", "head_pitch"]
        while(True):
            time.sleep(0.5)
            # пример получения данных из включенного и существующего сенсора
            # print("ball: ", self.get_sensor("BALL"))
            #print("gps_body: ", self.get_sensor("gps_body"))
            # print(self.get_ball())


if __name__ == '__main__':
    manager = CommunicationManager(1, '127.0.0.1', 10001, time_step = 20)
    # инициализация сенсоров
    

    
    while (True):
        # pass
        time.sleep(0.5)
        # print("IMU: ", manager.get_imu_body())
        # print(manager.current_time)
        #print("get_localization: ", manager.get_localization())
        print("get_ball: ", manager.get_ball())
        # print("get_imu: ", manager.get_imu_body())
        # manager.send_servos({"head_yaw": 0, "head_pitch": 0})
        # print(manager.current_time)
    # manager = CommunicationManager(1, '127.0.0.1', 10001, time_step = 20)
    # # инициализация сенсоров
    # while (True):
    #     # pass
    #     # time.sleep(0.5)
    #     print("IMU: ", manager.get_imu_body())
    #     print(manager.current_time)
    #     print("get_localization: ", manager.get_localization())
    #     print("ball: ", manager.get_ball())
    #     print("opp: ", manager.get_opponents())
    #     print("mates: ", manager.get_teammates())

    #     manager.send_servos({"head_pitch": -0.3})
    #     print(manager.current_time)
        

        # print("Time: ", manager.get_time())
        
    # th2.start()
    manager.thread.join()
    # th2.join