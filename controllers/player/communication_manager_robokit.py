"""[summary]
Returns:
[type]: [description]
"""
import queue
import time
import logging
from threading import Thread
import logging

# logging.basicConfig(filename='cm_robokit.txt', encoding="utf-8", level=logging.DEBUG)

from robot_client import RobotClient


class CommunicationManager():
    """[summary]
    """
    def __init__(self, maxsize=1, host='127.0.0.1', port=10001, team_color="RED", player_number = 1, time_step = 15):
        verbosity = 4
        self.client = RobotClient(host, port, verbosity)
        self.client.connect_client()
        self.maxsize = maxsize
        self.messages = queue.Queue(maxsize)
        self.sensors = {}
        self.robot_color = team_color
        self.robot_number = player_number
        self.time_step = time_step

        self.current_time = 0

        self.sensor_time_step = time_step * 4
        #self.sensor_time_step = 5

        sensors = {"left_knee_sensor": self.sensor_time_step, "right_knee_sensor": self.sensor_time_step,
                    "left_ankle_pitch_sensor": self.sensor_time_step, "right_ankle_pitch_sensor": self.sensor_time_step,
                    "right_hip_pitch_sensor": self.sensor_time_step, "left_hip_pitch_sensor": self.sensor_time_step,
                    "head_pitch_sensor": self.sensor_time_step, "head_yaw_sensor": self.sensor_time_step,
                    "imu_body": self.time_step, "recognition": self.sensor_time_step, "gps_body": self.sensor_time_step}
        # sensors = {"gps_body": 5, "imu_head": 5, "imu_body": 5,  "camera": 20}#
        self.enable_sensors(sensors)
        self.thread = Thread(target = self.run)
        # th2 = Thread(target=manager.test_run)
        #manager.run()
        self.thread.start()
        

    def enable_sensors(self, sensors) -> None:
        for sensor in sensors:
            self.client.initial(sensor, sensors[sensor])
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
            self.sensors[name].put(value_dict)
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
            positions = self.messages.get()
            logging.debug("Sending...")
            logging.debug(f"Time: {self.current_time}")
            logging.debug(positions)
            self.client.send_request("positions", positions)

    def update_history(self, message):
        for sensor in message:
            if (sensor == "time"):
                delta = message[sensor]['sim time'] - self.current_time
                if delta > 5:
                    print(f"WARNING! Large protobuf time rx delta = {delta}")                
                self.current_time = message[sensor]['sim time']
            if self.sensors[sensor].full():
                self.sensors[sensor].get()
                self.sensors[sensor].put(message[sensor])
            else:
                self.sensors[sensor].put(message[sensor])

    def time_sleep(self, t = 0.001)->None:
        print(f"Emulating delay of {t*1000} ms")
        start_time = self.current_time
        while (self.current_time - start_time < t * 1000):
            time.sleep(0.001)

    def get_imu_body(self):
        # self.last_imu_body = self.get_sensor("imu_body")
        self.time_sleep()
        return self.get_sensor("imu_body")

    def get_imu_head(self):
        self.time_sleep()
        return self.get_sensor("imu_head")

    def get_localization(self):
        self.time_sleep(0.5)
        return self.get_sensor("gps_body")

    def get_ball(self):
        self.time_sleep(0.1)
        return self.get_sensor("BALL")

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
        logging.debug("Getting servo commands:")
        logging.debug(f"Time: {self.current_time}")
        logging.debug(data)

        self.add_to_queue((data, {}))
        return 0 

        
    def run(self):
        while(True):
            do_not_block = True
            if do_not_block:
                # Sending/receiving protobuf in non-blocking way 
                # If we have any data to send - do sending
                # If full packet data is ready in socket - receive it, otherwise switch to check if sending is needed
                if not self.messages.empty():
                    self.send_message()
                message = self.client.receive2()
                if message:
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
        # time.sleep(0.5)
        # print("IMU: ", manager.get_imu_body())
        print(manager.current_time)
        #print("get_localization: ", manager.get_localization())
        print("get_ball: ", manager.get_ball())
        print("get_imu: ", manager.get_imu_body())
        manager.send_servos({"head_yaw": 1, "head_pitch": 1})
        print(manager.current_time)
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