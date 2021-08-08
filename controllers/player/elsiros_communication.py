import queue
from robot_client import RobotClient
import time

class CommunicationManager():
    def __init__(self, maxsize=5):
        host = '127.0.0.1'
        port = 10001
        verbosity = 4
        self.client = RobotClient(host, port, verbosity)
        self.client.connect_client()
        self.messages = queue.Queue(maxsize)
        self.actual = {}

    def get_imu(self):
        pass

    def get_position(self):
        if len(self.actual) > 0 and self.actual['gps']:
            return [self.actual['gps'][0].value.X, self.actual['gps'][0].value.Y]
        else:
            return 0

    def get_image(self):
        return

    def get_ball(self):
        pass

    def get_servo(self):
        pass

    def set_servo(self):
        pass

    def set_servos(self, positions):
        pass

    def parse_message(self):
        pass

    def update_history(self, message):
        if self.messages.full():
            self.messages.get()
            self.messages.put(message)
        else:
            self.messages.put(message)
        self.actual = message

    def update_queue(self):
        pass

    def run(self):
        self.client.send_request("positions", {"Head":-1.5, "Neck":-1.6})
        
        while(True):
            self.client.send_request()
            message = self.client.receive()
            self.update_history(message)
            print(self.get_position())


if __name__ == '__main__':
    manager = CommunicationManager()
    manager.run()
