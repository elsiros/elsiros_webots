"""[summary]
Returns:
[type]: [description]
"""
import queue
import time
import logging
from threading import Thread

from robot_client import RobotClient


class CommunicationManager():
    """[summary]
    """
    def __init__(self, maxsize=1, host='127.0.0.1', port=10001):
        verbosity = 4
        self.client = RobotClient(host, port, verbosity)
        self.client.connect_client()
        self.maxsize = maxsize
        self.messages = queue.Queue(maxsize)
        self.sensors = {}

    def enable_sensors(self, sensors) -> None:
        for sensor in sensors:
            self.client.add_initial_sensor(sensor, sensors[sensor])
            if sensor == "recognition":
                self.sensors.update({"BALL": queue.Queue(self.maxsize)})
                self.sensors.update({"RED_PLAYER_1": queue.Queue(self.maxsize)})
                self.sensors.update({"RED_PLAYER_2": queue.Queue(self.maxsize)})
                self.sensors.update({"BLUE_PLAYER_1": queue.Queue(self.maxsize)})
                self.sensors.update({"BLUE_PLAYER_2": queue.Queue(self.maxsize)})

            self.sensors.update({str(sensor): queue.Queue(self.maxsize)})
        self.sensors.update({"time": queue.Queue(1)})
        self.client.send_request("init")

    def get_sensor(self, name) -> dict:
        """[summary]

        Args:
            name ([type]): [description]

        Returns:
            dict: [description]
        """
        value_dict = {}
        if not name in self.sensors:
            logging.error("sensor is not enable")
            #return "sensor is not enable"
        elif not self.sensors[name].empty():
            value_dict = self.sensors[name].get()
            #logging.warn("nothing in queue")
            #return False
        #else:
            #return self.sensors[name].get()
        return value_dict

    def add_to_queue(self, message):
        if self.messages.full():
            self.messages.get()
            self.messages.put(message)
        else:
            self.messages.put(message)

    def send_message(self):
        while(not self.messages.empty()):
            self.client.send_request("positions", self.messages.get())

    def update_history(self, message):
        for sensor in message:
            if self.sensors[sensor].full():
                self.sensors[sensor].get()
                self.sensors[sensor].put(message[sensor])
            else:
                self.sensors[sensor].put(message[sensor])

    def run(self):
        data = {"head_pitch": -1.0, "head_yaw": 0.9}

        self.add_to_queue(data)
        while(True):
            self.send_message()
            message = self.client.receive()
            # print(message)
            self.update_history(message)

    def test_run(self):
        # пример отправки данных серв
        self.WBservosList = ["right_ankle_roll", "right_ankle_pitch", "right_knee", "right_hip_pitch",
                             "right_hip_roll", "right_hip_yaw", "right_elbow_pitch", "right_shoulder_twirl",
                             "right_shoulder_roll", "right_shoulder_pitch", "pelvis_yaw", "left_ankle_roll",
                             "left_ankle_pitch", "left_knee", "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
                             "left_elbow_pitch", "left_shoulder_twirl", "left_shoulder_roll",
                             "left_shoulder_pitch", "head_yaw", "head_pitch"]
        while(True):
            time.sleep(0.02)
            # пример получения данных из включенного и существующего сенсора
            print("recognition: ", self.get_sensor("recognition"))
            #print("imu_head: ", self.get_sensor("imu_head"))
            #print(self.get_sensor("time"))
            print('imu_body =', self.get_sensor("imu_body"))
            print('gps_body =', self.get_sensor("gps_body"))
            #print(self.get_sensor("ball"))
            servo_data = {}
            for key in self.WBservosList:
                servo_data.update({key: 0})
            #self.add_to_queue(servo_data)


if __name__ == '__main__':
    manager = CommunicationManager(1, '127.0.0.1', 7001)
    # инициализация сенсоров
    sensors = {"gps_body": 5,"head_pitch_sensor": 5, "head_yaw_sensor": 5, "imu_body": 5, "recognition": 5}#
    # sensors = {"gps_body": 5, "imu_head": 5, "imu_body": 5,  "camera": 20}#
    manager.enable_sensors(sensors)

    th1 = Thread(target=manager.run)
    th2 = Thread(target=manager.test_run)
    #manager.run()
    th1.start()
    th2.start()
    th1.join
    th2.join
