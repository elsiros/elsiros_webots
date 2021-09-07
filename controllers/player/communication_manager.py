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
    def __init__(self, maxsize=5, host='127.0.0.1', port=10001):
        verbosity = 4
        self.client = RobotClient(host, port, verbosity)
        self.client.connect_client()
        self.maxsize = maxsize
        self.messages = queue.Queue(maxsize)
        self.sensors = {}

    def enable_sensors(self, sensors) -> None:
        """[summary]
        """
        for sensor in sensors:
            self.client.initial(sensor, sensors[sensor])
            if sensor == "camera":
                self.sensors.update({"ball": queue.Queue(self.maxsize)})
                self.sensors.update({"robot": queue.Queue(self.maxsize)})
            self.sensors.update({str(sensor): queue.Queue(self.maxsize)})
        self.sensors.update({"time": queue.Queue(1)})
        self.sensors.update({"messages": queue.Queue(10)})
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
        elif not self.sensors[name].empty():
            value_dict = self.sensors[name].get()
        else:
            logging.warning("Nothing in queue")
        return value_dict

    def add_to_queue(self, message):
        if self.messages.full():
            self.messages.get()
            self.messages.put(message)
        else:
            self.messages.put(message)

    def send_message(self):
        while not self.messages.empty():
            self.client.send_request("positions", self.messages.get())

    def update_history(self, message):
        for sensor in message:
            if self.sensors[sensor].full():
                self.sensors[sensor].get()
                self.sensors[sensor].put(message[sensor])
            else:
                self.sensors[sensor].put(message[sensor])

    def run(self):
        data = {"head_pitch": -1.5}
        self.add_to_queue(data)
        while True:
            self.send_message()
            message = self.client.receive()
            self.update_history(message)

    def test_run(self):
        # пример отправки данных серв
        while True:
            time.sleep(1)
            # пример получения данных из включенного и существующего сенсора
            print(self.get_sensor("messages"))


if __name__ == '__main__':
    manager = CommunicationManager(1, '127.0.0.1', 10001)
    # инициализация сенсоров
    ex_sensors = {"gps_body": 5, "camera": 20,"head_pitch_sensor": 5, "imu_head": 5, "imu_body": 5}#
    manager.enable_sensors(ex_sensors)

    th1 = Thread(target=manager.run)
    th2 = Thread(target=manager.test_run)
    #manager.run()
    th1.start()
    th2.start()
    th1.join
    th2.join
