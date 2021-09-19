""" Class that provides communication with simulator Webots.
"""
import time
from threading import Thread, Lock
from robot_client import RobotClient

from blurrer import Blurrer
from model_robokit import Model
import logging

class CommunicationManager():
    def __init__(self, maxsize=1, host='127.0.0.1', port=10001, logger = logging, team_color="RED", player_number=1, time_step=15):
        self.__client = RobotClient(host, port, logger)
        self.__client.connect_client()
        self.maxsize = maxsize
        self.__sensors = {}
        self.robot_color = team_color
        self.robot_number = player_number
        self.time_step = time_step
        self.tx_mutex = Lock()
        self.tx_message = {}
        self.__last_message = {}
        self.last_head_yaw = 0
        self.last_head_pitch = 0
        self.__blurrer = Blurrer()
        self.__model = Model(self.__blurrer)
        self.current_time = 0
        self.sensor_time_step = time_step * 4
        self.logger = logger
        sensors = {"imu_body": self.time_step, "recognition": self.sensor_time_step, "gps_body": self.sensor_time_step}
        self.enable_sensors(sensors)
        self.thread = Thread(target=self.run)
        self.thread.start()

    def enable_sensors(self, sensors) -> None:
        for sensor in sensors:
            self.__client.initial(sensor, sensors[sensor])
            if sensor == "recognition":
                self.__sensors.update({"BALL": {}})
                self.__sensors.update({"RED_PLAYER_1": {}})
                self.__sensors.update({"RED_PLAYER_2": {}})
                self.__sensors.update({"BLUE_PLAYER_1": {}})
                self.__sensors.update({"BLUE_PLAYER_2": {}})

            self.__sensors.update({str(sensor): {}})
        self.__sensors.update({"time": {}})
        self.__client.send_request("init")

    def __get_sensor(self, name) -> dict:
        return self.__sensors[name]

    def __send_message(self):
        self.tx_mutex.acquire()
        if self.tx_message:
            self.__client.send_request("positions", self.tx_message)
            self.tx_message = {}
        self.tx_mutex.release()

    def __update_history(self, message):
        for sensor in message:
            if sensor == "warnings":
                self.logger.warning(message[sensor])
            if sensor == "time":
                delta = message[sensor]['sim time'] - self.current_time
                if delta > 5:
                    self.logger.warning(f"WARNING! Large protobuf time rx delta = {delta}")
                self.current_time = message[sensor]['sim time']
            self.__sensors[sensor] = message[sensor]

    def __procces_object(self, name):
        blur_object = {}
        imu_body = self.__get_sensor("imu_body")
        gps_body = self.__get_sensor("gps_body")
        last_message = self.__last_message
        real_object = self.__get_sensor(name).copy()
        if real_object and imu_body and gps_body:
            position = real_object["position"]
            if position:
                self.__model.update_robot_state(gps_body, imu_body, last_message, self.last_head_pitch, self.last_head_yaw)
                proccessed_object_pos = self.__model.proccess_data(position[0], position[1])
                real_object["position"] = proccessed_object_pos
                blur_object = real_object
        return blur_object

    def time_sleep(self, t) -> None:
        """Emulate sleep according to simulation time.

        Args:
            t (float): time
        """

        self.logger.debug(f"Emulating delay of {t*1000} ms")
        start_time = self.current_time
        while (self.current_time - start_time < t * 1000):
            time.sleep(0.001)

    def get_imu_body(self) -> dict:
        """Provide last measurement from imu located in body. 
        Can be empty if 'imu body' sensor is not enabled or webots does not 
        sent us any measurement. Also contains simulation time of measurement.

        Returns:
            dict: {"position": [roll, pitch, yaw]}
        """

        res = self.__get_sensor("imu_body")
        self.logger.debug(res)
        return res

    def get_imu_head(self) -> dict:
        """Provide last measurement from imu located in head.
        Can be empty if 'imu_head' sensor is not enabled or webots does not 
        send us any measurement. Also contains simulation time of measurement.

        Returns:
            dict: {"position": [roll, pitch, yaw], "time": time} 
        """

        res = self.__get_sensor("imu_head")
        self.logger.debug(res)
        return res

    def get_localization(self) -> dict:
        """Provide blurred position of the robot on the field and confidence in
        this position ('consistency' - where 1 fully confident and 0 - have no confidence).
        Can be empty if 'gps_body' sensor is not enabled or webots does not 
        send us any measurement. Also contains simulation time of measurement.

        Returns:
            dict: {"position": [x, y, consistency], "time": time} 
        """
        res = {}
        self.time_sleep(0.5)
        res = self.__get_sensor("gps_body").copy()
        if res:
            pos = res["position"]
            res["position"] = self.__blurrer.loc(pos[0], pos[1])
        self.logger.debug(res)
        return res

    def get_ball(self) -> dict:
        """Provide blurred position of the ball relative to the robot.
        Can be empty if:
        1. 'recognition', 'gps_body' or 'imu_body' sensors are not enabled
        2. webots did not send us any measurement.
        3. robot does not stand upright position
        4. ball is not in the camera field of view (fov)

        Also contains simulation time of measurement.

        Returns:
            dict: {"position": [x, y], "time": time} 
        """
        self.time_sleep(0.1)
        res = self.__procces_object("BALL")
        self.logger.debug(res)
        return res

    def get_opponents(self) -> list:
        """Provide blurred positions of the opponents relative to the robot.
        Can be empty if:
            1. 'recognition', 'gps_body' or 'imu_body' sensors are not enabled
            2. webots did not send us any measurement.
            3. robot does not stand upright position
            4. opponent is not in the camera field of view (fov)

        Also contains simulation time of measurement.

        Returns:
            list: [{"position": [x1, y1], "time": time}, {"position": [x2, y2], "time": time}]
        """
        self.time_sleep(0.1)
        players = (1,2)
        color = "BLUE" if self.robot_color == "RED" else "RED"
        opponents = []
        for number in players:
            opponents.append(self.__procces_object(f"{color}_PLAYER_{number}"))

        self.logger.debug(opponents)
        return opponents        

    def get_mates(self) -> dict:
        """Provide blurred position of the mate relative to the robot.
        Can be empty if:
            1. 'recognition', 'gps_body' or 'imu_body' sensors are not enabled
            2. webots did not send us any measurement.
            3. robot does not stand upright position
            4. mate is not in the camera field of view (fov)

        Also contains simulation time of measurement.

        Returns:
            list: {"position": [x, y], "time": time}
        """
        self.time_sleep(0.1)
        number = 1 if self.robot_number == 2 else 2
        res = self.__procces_object(f"{self.robot_color}_PLAYER_{number}")
        self.logger.debug(res)
        return res

    def get_time(self) -> float:
        """Provide latest observed simulation time.

        Returns:
            float: simulation time
        """
        res = self.current_time
        self.logger.debug(res)
        return res

    def send_servos(self, data) -> None:
        """Add to message queue dict with listed servo names and angles in radians.
        List of posible servos:
        ["right_ankle_roll", "right_ankle_pitch", "right_knee", "right_hip_pitch",
        "right_hip_roll", "right_hip_yaw", "right_elbow_pitch", "right_shoulder_twirl",
        "right_shoulder_roll", "right_shoulder_pitch", "pelvis_yaw", "left_ankle_roll",
        "left_ankle_pitch", "left_knee", "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
        "left_elbow_pitch", "left_shoulder_twirl", "left_shoulder_roll",
        "left_shoulder_pitch", "head_yaw", "head_pitch"]

        Args:
            data (dict): {servo_name: servo_angle, ...}
        """
        self.tx_mutex.acquire()
        self.tx_message = data
        self.tx_mutex.release()

        # self.logger.debug(data)

        if "right_hip_yaw" in data.keys():
            self.__last_message = data

        if "head_yaw" in data.keys():
            self.last_head_yaw = data["head_yaw"]
        if "head_pitch" in data.keys():
            self.last_head_pitch = data["head_pitch"]

    def run(self):
        """Infinity cycle of sending and receiving messages.
        Should be launched in sepparet thread. Communication manager
        launch this func itself in constructor
        """
        while(True):
            do_not_block = True
            if do_not_block:
                # Sending/receiving protobuf in non-blocking way
                # If we have any data to send - do sending
                # If full packet data is ready in socket - receive it, otherwise switch to check if sending is needed
                self.__send_message()
                messages_list = self.__client.receive2()
                for message in messages_list:
                    self.__update_history(message)
            else:
                # Sending/receiving protobuf in blocking way:
                # wait for a message ready to be sent, then send it
                # Then wait for a message to be received, and receive it
                self.__send_message()
                message = []
                while not message:
                    message = self.__client.receive2()
                self.__update_history(message)
