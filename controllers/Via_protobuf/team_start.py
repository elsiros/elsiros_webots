import time
from threading import Thread
from communication_manager import CommunicationManager
#import dotenv
#import os
import main

#print(os.environ.get('Path1'))
#dotenv.load_dotenv(override=True)
#print(os.environ.get('Path1').replace(';', '\n'))
color = 'red'
number = 2
robot = CommunicationManager(5, '127.0.0.1', 10002)
sensors = {"gps_body": 2, "imu_head": 2, "imu_body": 2,  "camera": 20}
robot.enable_sensors(sensors)

th0 = Thread(target=robot.run)
th0.start()
th0.join
th1 = Thread(target=main.main, args=(robot, color, number))
th1.start()
th1.join
#main.main(robot, color, number)