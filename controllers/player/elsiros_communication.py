import queue
from robot_client import RobotClient
import time
from threading import Thread


class CommunicationManager():
    def __init__(self, host='127.0.0.1', port=10001, maxsize=5):
        verbosity = 4
        self.client = RobotClient(host, port, verbosity)
        self.client.connect_client()
        self.maxsize = maxsize
        self.messages = queue.Queue(maxsize)
        self.actual = {}
        self.sensors = {}

    def enable_sensors(self, sensors):
        for sensor in sensors:
            self.client.add_initial_sensor(sensor, sensors[sensor])
            self.sensors.update({str(sensor): queue.Queue(self.maxsize)})
        self.client.send_request("init")

    def get_sensor(self, name):
        if not name in self.sensors:
            return "sensor is not enable"
        if self.sensors[name].empty():
            return False
        else:
            return self.sensors[name].get()

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
        # инициализация сенсоров
        sensors = {"gps_body": 5, "camera": 5, "NeckS": 5}
        self.enable_sensors(sensors)

        while(True):
            self.send_message()
            message = self.client.receive()
            self.update_history(message)

    def test_run(self):
        # пример отправки данных серв
        data = {"Head": -1.5, "Neck": -1.6}
        self.add_to_queue(data)
        while(True):

            time.sleep(1)
            # пример получения данных из включенного и существующего сенсора
            print(self.get_sensor("NeckS"))

    def execute(self):
        thread = Thread(target=self.run)
        thread.start()
        thread.join()

if __name__ == '__main__':
    manager = CommunicationManager()
    th1 = Thread(target=manager.run)
    th2 = Thread(target=manager.test_run)
    # manager.run()
    th1.start()
    th2.start()
    th1.join
    th2.join
