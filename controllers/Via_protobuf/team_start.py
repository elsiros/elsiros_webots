import time
from threading import Thread
from communication_manager import CommunicationManager
#import main_pb

def process_11():
    execfile("main_pb_11.py")

def process_12():
    execfile("main_pb_12.py")

t11 = Thread(target = process_11, daemon = True)
t11.start()
t11.join()

t12 = Thread(target = process_12, daemon = True)
t12.start()
t12.join()