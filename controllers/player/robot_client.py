"""[summary]
Returns:
[type]: [description]
"""
import socket
import time
import logging

from message_manager import MessageManager


class RobotClient():
    """[summary]
    """
    def __init__(self, host="127.0.0.1", port=1, logger=logging,
                 max_attempts=20, wait_time=1):
        """[summary]

        Args:
            host (str): The host name. Defaults to 127.0.0.1.
            port (int): Port to connect to.
            max_attempts (int): Maximum number of attempts to connect. Defaults to 20.
            wait_time (float, optional): Time between attempts to connect, in seconds. Defaults to 1.
        """
        self.logger = logger
        self.host = host
        self.port = port
        self.max_attempts = max_attempts
        self.wait_time = wait_time
        self.message_manager = MessageManager(self.logger)
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
            self.logger.warning("Cannot create socket. \
                                 Caught exception socket.error %s\n", msg)
            return False
        attempt = 1
        connected = False
        for attempt in range(self.max_attempts):
            try:
                self.socket.connect((self.host, self.port))
                connected = True
                break
            except socket.error as msg:
                self.logger.warning("Failed to connect to %s:%s %s \
                                attempt  of %s.", self.host, self.port,
                                attempt, self.max_attempts)
                self.logger.warning("Caught exception socket.error : %s", msg)
                time.sleep(self.wait_time)

        if not connected:
            self.logger.warning("Failed to connect after \
                    %s attempts. Giving up on connection", attempt)
            self.disconnect_client()
            return False

        # Receiving the 'welcome message'
        welcome_message = self.socket.recv(8)
        # self.logger.info("Welcome message: ", welcome_message.decode("utf-8"))
        if welcome_message != b'Welcome\x00':
            self.logger.warning("Incorrect welcom message")
            if welcome_message == b'Refused\x00':
                self.logger.warning("Connection refused")
            else:
                self.logger.warning("Received unknown answer from server: %s",
                                welcome_message.decode("utf-8"))
            self.disconnect_client()
            return False
        # self.logger.info("Connected to ", self.host, self.port)
        return True

    @staticmethod
    def print_messages(msg):
        """[summary]
        """
        print(msg)

    def disconnect_client(self) -> None:
        """
        Сloses the client-side connection
        """
        self.logger.info("Disconnect client")
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
        # self.logger.debug("Sending byte message: %s", message)
        try:
            self.socket.send(message)
        except socket.error as msg:
            self.logger.error("Can't send request with error: %s", msg)

    def initial(self, sensor_name, sensor_time):
        self.message_manager.add_initial_request(sensor_name, sensor_time)

    def receive(self):
        """!NOT RECOMENDENT TO USE!

        """
        content_size = self.socket.recv(self.message_manager.get_size())
        buffer_size = self.message_manager.get_answer_size(content_size)
        # self.logger.debug("Recrive %s bytes size", buffer_size)
        data = self.socket.recv(buffer_size)
        # self.logger.debug("Receive %s bytes message", data)
        return self.message_manager.parse_answer_message(data)

    def receive2(self):
        messages_list = []
        chunk = self.socket.recv(1024)
        self.rx_buf.extend(chunk)
        # self.logger.debug("Receive %s chunk", chunk)
        header_size = self.message_manager.get_size()
        # self.logger.debug("Receive %s header size", header_size)
        while True:
            if self.rx_wait_for_data == False:
                if len(self.rx_buf) >= header_size:
                    self.rx_wait_for_data = True
                    header = self.rx_buf[:header_size]
                    self.rx_buf = self.rx_buf[header_size:]
                    self.rx_expected_data_size = self.message_manager.get_answer_size(header)
                else:
                    # not enough data even for header
                    break
            if self.rx_wait_for_data == True:
                if len(self.rx_buf) >= self.rx_expected_data_size:
                    self.rx_wait_for_data = False
                    data = self.rx_buf[:self.rx_expected_data_size]
                    self.rx_buf = self.rx_buf[self.rx_expected_data_size:]
                    messages_list.append(
                        self.message_manager.parse_answer_message(data))
                else:
                    # not enough data for message body
                    break
        # self.logger.debug("Receive %s bytes message", messages_list)
        return messages_list
