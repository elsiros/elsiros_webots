from communication_manager import CommunicationManager

ports = [10001, 10002, 10021]
sensors = {"left_knee_sensor": 5, "right_knee_sensor": 5,
            "left_ankle_pitch_sensor": 5, "right_ankle_pitch_sensor": 5,
            "right_hip_pitch_sensor": 5, "left_hip_pitch_sensor": 5,  
            "gps_body": 5,"head_pitch_sensor": 5, "head_yaw_sensor": 5, 
            "imu_body": 5, "recognition": 5}

# managers = []
threads = []
for port in ports:
    print(f"Port: {port}")
    manager = CommunicationManager(1, '127.0.0.1', port)
    manager.enable_sensors(sensors)
    th = Thread(target=manager.run)
    th.start()
    threads.append(th)
    
for el in threads:
    el.join