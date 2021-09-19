"""
Сlass operates with protobuff messages. used to create and parse messages.

"""
import logging
# TODO: Can't add google.protobuf to exe
# from google.protobuf import text_format

import messages_pb2


class MessageManager():
    def __init__(self, logger = logging,  head_buffer_size=4):
        """
        Args:
            head_buffer_size (int): Value of heder byte buffer. Defaults to 4.
        """
        self.logger = logger
        self.size = head_buffer_size
        self.init_request = None

    def get_size(self):
        """
        Returns:
            int: Value of heder byte buffer.
        """
        return self.size

    @staticmethod
    def create_requests_message():
        """Create Empty protobuf class instance for request message.

        Returns:
            messages_pb2: Empty protobuf class instance.
        """
        return messages_pb2.ActuatorRequests()

    @staticmethod
    def create_answer_message():
        """Create Empty protobuf class instance for answer message.

        Returns:
            messages_pb2: Empty protobuf class instance.
        """
        return messages_pb2.SensorMeasurements()

    @staticmethod
    def build_request_from_file(path):
        """Parsing data from message file to protobuf message instance.

        Args:
            path (string): path to filename.txt with message.

        Returns:
            messages_pb2: protobuf class instance of filled
            message from file.
        """
        request = messages_pb2.ActuatorRequests()
        # TODO: Can't add google.protobuf to exe
        # with open(path, 'r',  encoding="utf-8") as actuator_requests:
        #     text_format.Parse(actuator_requests.read(), request)
        return request

    def build_request_positions(self, positions):
        """Сreating an instance of the protobuff class
        and fills it with the values ​​of the actuators

        Args:
            positions (dict): key - servo name and values ​​- position.
        Returns:
            messages_pb2: protobuf class instance of filled
            message with servos.
        """
        request = messages_pb2.ActuatorRequests()
        for pos in positions:
            motor = request.motor_positions.add()
            motor.name = pos
            motor.position = positions[pos]
        return self.generate_message(request)

    @staticmethod
    def generate_message(message):
        """Generate bytes string for sending message.

        Args:
            message (messages_pb2): protobuf class instance of filled
            message.

        Returns:
            bytes: bytes string of message.
        """
        return message.ByteSize().to_bytes(4, byteorder='big', signed=False) \
            + message.SerializeToString()

    def message_from_file(self, path):
        """
        Function process the protobuff message. Measurement values
        of sensors, messages from player.exe and webots.
        Received messages are placed in the dictionary
        Args:
            path ([string]): path to filename.txt with default message.

        Returns:
            messages_pb2: protobuf class instance, with values ​​from file.
        """
        return self.generate_message(self.build_request_from_file(path))

    def get_answer_size(self, content_size):
        """
        Сalculating message size from header bytes

        Args:
            content_size (bytes): Byte size of answer message.

        Returns:
            int: Size of answer message.
        """
        size = int.from_bytes(content_size, byteorder='big', signed=False)
        # self.logger.debug("Byte size of received messages: %d", size)
        return size

    def add_initial_request(self, sensor_name, sensor_time):
        """Generate bytes string for sending message.

        Args:
            sensor_name (string): protobuf class instance of filled
            message.

        Returns:
            bytes: bytes string of message.
        """
        if self.init_request is None:
            self.init_request = messages_pb2.ActuatorRequests()
        sensor = self.init_request.sensor_time_steps.add()
        sensor.name = sensor_name
        sensor.timeStep = sensor_time

    def build_initial_request(self):
        """Generate bytes string for initialization message.

        Returns:
            bytes: bytes string of message.
        """
        return self.generate_message(self.init_request)

    def parse_answer_message(self, data):
        """Parsing answer message from byte array to dict with measurements

        Args:
            data ([type]): [description]

        Returns:
            [type]: [description]
        """
        message = messages_pb2.SensorMeasurements()
        message.ParseFromString(data)
        return self.parse_message(message)

    @staticmethod
    def parse_message(message) -> dict:
        """
        Function process the protobuff message. Measurement values
        of sensors, messages from player.exe and webots.
        Received messages are placed in the dictionary
        Args:
            message (messages_pb2): protobuf class instance
            of new message with filled or unfilled.

        Returns:
            dict: dict with keys of names sensors
        """
        parse_message = {}
        parse_message.update(
            {
                "time":
                {
                    "unix time": message.real_time,
                    "sim time": message.time
                }
            })
        for sensor in message.accelerometers:
            parse_message.update(
                {
                    sensor.name:
                    {
                        "position":
                        [
                            sensor.value.X,
                            sensor.value.Y,
                            sensor.value.Z
                        ],
                        "time": message.time
                    }
                })
        for sensor in message.cameras:
            parse_message.update(
                {
                    sensor.name:
                        {
                            "width": sensor.width,
                            "height": sensor.height,
                            "quality": sensor.quality,
                            "image": sensor.image,
                            "time": message.time
                        }
                })
        for sensor in message.position_sensors:
            parse_message.update(
                {
                    sensor.name:
                    {
                        "position": sensor.value,
                        "time": message.time
                    }
                })
        for sensor in message.gyros:
            parse_message.update(
                {
                    sensor.name:
                    {
                        "position":
                        [
                            sensor.value.X,
                            sensor.value.Y,
                            sensor.value.Z
                        ],
                        "time": message.time
                    }
                })
        for sensor in message.gps:
            parse_message.update(
                {
                    sensor.name:
                    {
                        "position": [sensor.value.X, sensor.value.Y],
                        "time": message.time
                    }
                })
        if hasattr(message, "objects"):
            for sensor in message.objects:
                parse_message.update(
                    {
                        sensor.name:
                            {
                                "position": [sensor.value.X, sensor.value.Y],
                                "time": message.time
                            }
                    })
        for sensor in message.imu:
            parse_message.update(
                {
                    sensor.name:
                    {
                        "position":
                        [
                            sensor.angles.roll,
                            sensor.angles.pitch,
                            sensor.angles.yaw
                        ],
                        "time":
                            message.time
                    }
                })
        for sensor in message.messages:
            parse_message.update(
                {
                    "warnings":
                    {
                        "message_type": sensor.message_type,
                        "text": sensor.text,
                        "time": message.time
                    }
                })
        return parse_message
