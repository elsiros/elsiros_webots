from communication_manager_robokit import CommunicationManager
from threading import Thread
import logging 

logging.basicConfig(level=logging.DEBUG)

ports = [10001]

# managers = []
threads = []
managers = []
for port in ports:
    print(f"Port: {port}")
    manager = CommunicationManager(1, '127.0.0.1', port)
    managers.append(manager)

while True:
    for port, manager in zip(ports, managers):
        print(f"[PORT: {port}] Ball pos: {manager.get_ball()}")
        print(f"[PORT: {port}] Localisation: {manager.get_localization()}")
        print(f"[PORT: {port}] Opponents pos: {manager.get_opponents()}")
        print(f"[PORT: {port}] Mates pos: {manager.get_mates()}")

for manager in managers:
    manager.thread.join()
