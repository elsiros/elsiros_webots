"""[summary]
Returns:
[type]: [description]
"""
import time
from threading import Thread, Lock
from robot_client import RobotClient
from blurrer import Blurrer
from model_robokit import Model

class CommunicationManager():
    def __init__(self, maxsize=1, host='127.0.0.1', port=10001, team_color="RED", player_number=1, time_step=15):
        verbosity = 4
        self.__client = RobotClient(host, port, verbosity)
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
        self.__model = Model(self, self.__blurrer)
        self.current_time = 0
        self.sensor_time_step = time_step * 4

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
            if (sensor == "time"):
                delta = message[sensor]['sim time'] - self.current_time
                if delta > 5:
                    pass
                    #print(f"WARNING! Large protobuf time rx delta = {delta}")
                self.current_time = message[sensor]['sim time']
            self.__sensors[sensor] = message[sensor]

    def time_sleep(self, t=0) -> None:
        # print(f"Emulating delay of {t*1000} ms")
        start_time = self.current_time
        while (self.current_time - start_time < t * 1000):
            time.sleep(0.001)

    def get_imu_body(self):
        return self.__get_sensor("imu_body")

    def get_imu_head(self):
        return self.__get_sensor("imu_head")

    def get_localization(self):
        self.time_sleep(0.5)
        res = self.__get_sensor("gps_body").copy()
        pos = res["position"]
        res["position"] = self.__blurrer.loc(pos[0], pos[1])
        return res

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

    def get_ball(self):
        self.time_sleep(0.1)
        return self.__procces_object("BALL")

    def get_opponents(self):
        self.time_sleep(0.1)
        players = (1,2)
        color = "BLUE" if self.robot_color == "RED" else "RED"
        opponents = []
        for number in players:
            opponents.append(self.__procces_object(f"{color}_PLAYER_{number}"))
        return opponents        

    def get_mates(self):
        self.time_sleep(0.1)
        number = 1 if self.robot_number == 2 else 2
        return self.__procces_object(f"{self.robot_color}_PLAYER_{number}")

    def get_time(self):
        return self.__get_sensor("time")

    def send_servos(self, data={}):
        self.tx_mutex.acquire()
        self.tx_message = data
        self.tx_mutex.release()

        if "right_hip_yaw" in data.keys():
            self.__last_message = data

        if "head_yaw" in data.keys():
            self.last_head_yaw = data["head_yaw"]
        if "head_pitch" in data.keys():
            self.last_head_pitch = data["head_pitch"]

    def run(self):
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
