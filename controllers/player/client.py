from google.protobuf import text_format
import messages_pb2

import socket
import sys
import time
from robot_client import RobotClient

def usage(error_msg = ""):
    if (error_msg):
        print("Invalid call: %s\n", error_msg)
    print("Usage: client [-v verbosity_level] <host> <port>\n")

def run():
    port = -1
    arg_idx = 1
    verbosity = 3
    host = ""
    while (arg_idx < len(sys.argv)):
        current_arg = sys.argv[arg_idx]
        if (current_arg[0] == '-'):
            if (current_arg == "-v"):
                if (arg_idx + 1 >= len(sys.argv)):
                    usage("Missing value for verbosity")
                verbosity = int(sys.argv[arg_idx + 1])
                arg_idx+=1
            else:
                usage()
        elif (len(host) == 0):
            host = current_arg
        elif (port == -1):
            port = int(current_arg)
            if (port < 0):
                usage("Unexpected negative value for port: " + current_arg)
            else:
                usage("Unexpected additional argument: " + current_arg)
        arg_idx+=1
    
  
    if (port == -1):
        usage("Missing arguments")

    host = '127.0.0.1'
    port = 10001
    verbosity = 4
    robot_client = RobotClient(host, port, verbosity)
    robot_client.connect_client()
    #if(True):
    while(True):
        #try:
        actuator_request = robot_client.build_request_message("actuator_requests.txt")
        actuator_request = messages_pb2.ActuatorRequests()
        motor = actuator_request.motor_positions.add()
        motor.name = "head_yaw"
        motor.position = 1.7
      
      
        robot_client.send_request(actuator_request)
        time.sleep(0.1)
        #robot_client.receive()
        #except Exception as ex:
        #    print("Runtime error: " + str(ex))
    
    
    
    
    return 0
    while(True):
        if (True):
        #try:
            request = messages_pb2.ActuatorRequests()
            with open("actuator_requests.txt") as actuator_requests:
                text_format.Parse(actuator_requests.read(), request)
            
            robot_client.send(request.SerializeToString())
            
            data = robot_client.recv(1024)
            print(data)
            sensors = messages_pb2.SensorMeasurements()
            sensors.ParseFromString(data)
            
            print(sensors.cameras)
        ##except Exception as ex:
        #    print("Runtime error: " + str(ex))

if __name__ == '__main__':
    
    run()