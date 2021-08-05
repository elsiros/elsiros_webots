import socket
import sys
import time

from google.protobuf.internal.decoder import _DecodeVarint32
from google.protobuf.message import DecodeError
from google.protobuf import text_format
import messages_pb2

max_attempts = 20
wait_time_sec = 1


def usage(error_msg=""):
    if (error_msg):
        print("Invalid call: %s\n", error_msg)
    print("Usage: client [-v verbosity_level] <host> <port>\n")


class RobotClient():
    def __init__(self, host='',  port=-1, verbosity=3):
        self.host = host
        self.port = port
        self.verbosity = verbosity

    def connect_client(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except:
            if (self.verbosity > 0):
                print("Cannot create socket\n")
                return False
        attempt = 1
        connected = False
        for attempt in range(max_attempts):
            try:
                self.socket.connect((self.host, self.port))
                connected = True
                break
            except:
                print("Failed to connect to ", self.host, " : ",
                      self.port, " attempt ",  attempt, " of ", max_attempts)
                time.sleep(wait_time_sec)

        if not connected:
            if (self.verbosity > 0):
                print("Failed to connect after ", attempt,
                      "attempts. Giving up on connection")
            self.disconnect_client()
            return False

        # Receiving the 'welcome message'
        welcome_message = self.socket.recv(8)
        if (self.verbosity >= 4):
            print("Welcome message:", welcome_message.decode("utf-8"))
        if welcome_message != b'Welcome\x00':
            if (self.verbosity > 0):
                if welcome_message == b'Refused\x00':
                    print("Connection refused")
                else:
                    print("Received unknown answer from server: ",
                          welcome_message.decode("utf-8"))
            self.disconnect_client()
            return False

        if (self.verbosity >= 2):
            print("Connected to ", str(self.host) + " :", self.port)
        return True

    def printMessages():
        pass

    def disconnect_client(self):
        self.socket.close()

    def send_request(self, actuator_request):
        #try:
        self.socket.send(actuator_request.SerializeToString())
        print("send servos")
        #except:
        #    print("Can't send request")

    def build_request_message(self, path):
        request = messages_pb2.ActuatorRequests()
        with open(path) as actuator_requests:
            text_format.Parse(actuator_requests.read(), request)
        return request

    def receive(self):
        sensors_measurements = messages_pb2.SensorMeasurements()
        
        data = self.socket.recv(1024)
        #print(data)

        data = self.socket.recv(1024)# read file as string
        decoder = _DecodeVarint32          # get a varint32 decoder
                                         # others are available in varint.py

        next_pos, pos = 0, 0
        while pos < len(data):
            sensors_measurements = messages_pb2.SensorMeasurements()                    # your message type
            next_pos, pos = decoder(data, pos)
            sensors_measurements.ParseFromString(data[pos:pos + next_pos])
            #print(sensors_measurements.objects)
            pos += next_pos     
    # use parsed message

    
        #data = ''.join(self.socket.recv_multipart())
        #time.sleep(0.1)
        #sensors_measurements.ParseFromString(data)
        
        #print(sensors_measurements.gps)


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
                arg_idx += 1
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
        arg_idx += 1

    if (port == -1):
        usage("Missing arguments")

    
