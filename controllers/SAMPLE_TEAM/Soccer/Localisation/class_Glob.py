"""
The module is designed by team Robokit of Phystech Lyceum and team Starkit
of MIPT under mentorship of A. Babaev.

This module is used to store variables which are used in many classes

"""


import json, array, math

class Glob:
    def __init__(self, simulation, current_work_directory):
        self.COLUMNS = 18
        self.ROWS = 13
        self.current_work_directory = current_work_directory
        self.strategy_data = array.array('b',(0 for i in range(self.COLUMNS * self.ROWS * 2)))
        self.SIMULATION = simulation     # 0 - Simulation without physics, 1 - Simulation synchronous with physics, 3 - Simulation streaming with physics
        self.ball_coord =[0.0,0.0]
        self.pf_coord = [0.0,0.0,0.0]
        self.obstacles = []
        import socket
        with open(current_work_directory / "Init_params" / "Sim_landmarks.json", "r") as f:
            landmarks = json.loads(f.read())
        with open(current_work_directory / "Init_params" / "Sim_params.json", "r") as f:
            self.params = json.loads(f.read())
        with open(current_work_directory / "Init_params" / "wifi_params.json", "r") as f:
            self.wifi_params = json.loads(f.read())
        if self.wifi_params['WIFI_IS_ON']:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.target_wifi_address = (self.wifi_params['HOST'], self.wifi_params['PORT'])
        self.first_step_yield = (19 * self.params['RUN_TEST_10_STEPS'] - 9 * self.params['RUN_TEST_20_STEPS']) / 10
        self.cycle_step_yield = ( self.params['RUN_TEST_20_STEPS'] - self.params['RUN_TEST_10_STEPS']) / 10
        self.side_step_right_yield = self.params['SIDE_STEP_RIGHT_TEST_RESULT'] / 20
        self.side_step_left_yield = self.params['SIDE_STEP_LEFT_TEST_RESULT'] / 20
        self.landmarks = landmarks
        self.import_strategy_data(current_work_directory)
        self.obstacleAvoidanceIsOn =  self.params['ObstacleAvoidanceIsOn']
        #self.imu_drift_correction = 0
        #self.imu_drift_last_correction_time = 0

    def import_strategy_data(self, current_work_directory):
        with open(current_work_directory / "Init_params" / "strategy_data.json", "r") as f:
            loaded_Dict = json.loads(f.read())
        if loaded_Dict.get('strategy_data') != None:
            strategy_data = loaded_Dict['strategy_data']
        for column in range(self.COLUMNS):
            for row in range(self.ROWS):
                index1 = column * self.ROWS + row
                power = strategy_data[index1][2]
                yaw = int(strategy_data[index1][3] * 40)  # yaw in radians multiplied by 40
                self.strategy_data[index1*2] = power
                self.strategy_data[index1*2+1] = yaw



