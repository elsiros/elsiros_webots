from google.protobuf import text_format
import messages_pb2

import socket
import sys


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

    #host = '127.0.0.1'
    #port = 10001
    
    
    robot_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    robot_client.connect((host, port))
    
    while(True):
        
        try:
            request = messages_pb2.ActuatorRequests()
            with open("actuator_requests.txt") as actuator_requests:
                text_format.Parse(actuator_requests.read(), request)
            
            robot_client.send(request.SerializeToString())
            data = robot_client.recv(1024)
            sensors = messages_pb2.SensorMeasurements()
            ans = sensors.ParseFromString(data)
            
            print(ans)
        except Exception as ex:
            print("Runtime error: " + str(ex))

if __name__ == '__main__':
    
    run()