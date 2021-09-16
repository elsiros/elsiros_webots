"""[summary]
Returns:
[type]: [description]
"""
import socket
import time
import logging

from message_manager import MessageManager


def usage(error_msg=""):
    if (error_msg):
        print("Invalid call: %s\n", error_msg)
    print("Usage: client [-v verbosity_level] <host> <port>\n")


class RobotClient():
    """[summary]
    """
    def __init__(self, host='', port=-1, verbosity=3, max_attempts=20, wait_time_sec=1):
        self.host = host
        self.port = port
        self.verbosity = verbosity
        self.max_attempts = max_attempts
        self.wait_time_sec = wait_time_sec
        self.message_manager = MessageManager()
        self.socket = None
        self.rx_buf = bytearray()
        self.rx_wait_for_data = False
        self.rx_expected_data_size = 0

    def connect_client(self):
        """[summary]

        Returns:
            [type]: [description]
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error as msg:
            if self.verbosity > 0:
                logging.warning("Cannot create socket. Caught exception socket.error %s\n", msg)
                return False
        attempt = 1
        connected = False
        for attempt in range(self.max_attempts):
            try:
                self.socket.connect((self.host, self.port))
                connected = True
                break
            except socket.error as msg:
                logging.warning("Failed to connect to %s:%s %s attempt  of %s.",
                                self.host, self.port, attempt, self.max_attempts)
                logging.warning("Caught exception socket.error : %s", msg)
                time.sleep(self.wait_time_sec)

        if not connected:
            if self.verbosity > 0:
                logging.warning("Failed to connect after %s attempts. Giving up on connection", attempt)
            self.disconnect_client()
            return False

        # Receiving the 'welcome message'
        welcome_message = self.socket.recv(8)
        if self.verbosity >= 4:
            print("Welcome message: ", welcome_message.decode("utf-8"))
        if welcome_message != b'Welcome\x00':
            if self.verbosity > 0:
                if welcome_message == b'Refused\x00':
                    logging.warning("Connection refused")
                else:
                    logging.warning("Received unknown answer from server: %s",
                          welcome_message.decode("utf-8"))
            self.disconnect_client()
            return False

        if self.verbosity >= 2:
            print("Connected to ", self.host, self.port)
        return True

    @staticmethod
    def print_messages(msg):
        """[summary]
        """
        print(msg)

    def disconnect_client(self)->None:
        """[summary]
        """
        self.socket.close()

    def send_request(self, message_type="default", positions={}):
        """[summary]
        """
        if message_type == "default":
            message = self.message_manager.message_from_file(
                "actuator_requests.txt")
        elif message_type == "positions":
            message = self.message_manager.build_request_positions(positions)
        elif message_type == "init":
            message = self.message_manager.build_initial_request()
        # try:
        self.socket.send(message)
       # except:
        #    print("Can't send request")

    def initial(self, sensor_name, sensor_time):
        self.message_manager.add_initial_request(sensor_name, sensor_time)

    def receive(self):
        content_size = self.socket.recv(self.message_manager.get_size())
        buffer_size = self.message_manager.get_answer_size(content_size)
        data = self.socket.recv(buffer_size)
        #print('content_size:', str(content_size), 'buffer_size:', str(buffer_size))
        return self.message_manager.parse_answer_message(data)

    def receive2(self):
        chunk = self.socket.recv(1024)
        self.rx_buf.extend(chunk)
        if self.rx_wait_for_data == False:
            header_size = self.message_manager.get_size()
            if len(self.rx_buf) >= header_size:
                self.rx_wait_for_data = True
                header = self.rx_buf[:header_size]     
                self.rx_buf = self.rx_buf[header_size:]
                self.rx_expected_data_size = self.message_manager.get_answer_size(header)
        else:
            if len(self.rx_buf) >= self.rx_expected_data_size:
                self.rx_wait_for_data = False
                data = self.rx_buf[:self.rx_expected_data_size]     
                self.rx_buf = self.rx_buf[self.rx_expected_data_size:]  
                return self.message_manager.parse_answer_message(data)
        return []              
