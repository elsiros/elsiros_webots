import time
from threading import Thread
from communication_manager import CommunicationManager
import dotenv
import os

#print(os.environ.get('Path1'))
#dotenv.load_dotenv(override=True)
#print(os.environ.get('Path1').replace(';', '\n'))

manager = CommunicationManager(5, '127.0.0.1', 10001)
sensors = {"gps_body": 5, "imu_head": 5, "imu_body": 5}
manager.enable_sensors(sensors)

th1 = Thread(target=manager.run)
#th2 = Thread(target=manager.test_run)
th1.start()
#th2.start()
th1.join
#th2.join
while(True):
    time.sleep(1)
    # пример получения данных из включенного и существующего сенсора
    print(manager.get_sensor("ball"))

execfile("main.py")