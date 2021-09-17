"""[summary]

    Returns:
        [type]: [description]
"""
from google.protobuf import text_format

import messages_pb2


class MessageManager():
    """[summary]
    """
    def __init__(self, init_buffer_size=4):
        self.size = init_buffer_size
        self.init_request = None

    def get_size(self):
        """[summary]
        Returns:
            [type]: [description]
        """
        return self.size

    @staticmethod
    def create_requests_message():
        """[summary]

        Returns:
            [type]: [description]
        """
        return messages_pb2.ActuatorRequests()

    @staticmethod
    def create_answer_message():
        """[summary]

        Returns:
            [type]: [description]
        """
        return messages_pb2.SensorMeasurements()

    def build_request_from_file(self, path):
        """[summary]

        Args:
            path ([type]): [description]

        Returns:
            [type]: [description]
        """
        request = messages_pb2.ActuatorRequests()
        with open(path, 'r') as actuator_requests:
            text_format.Parse(actuator_requests.read(), request)
        return request

    def build_request_positions(self, positions):
        """[summary]

        Args:
            positions ([type]): [description]

        Returns:
            [type]: [description]
        """
       
        request = messages_pb2.ActuatorRequests()
        #for sen in positions[1]:
        #    sensor = request.sensor_time_steps.add()
        #    sensor.name = sen
        #    sensor.timeStep = positions[1][sen]
        for pos in positions:
            motor = request.motor_positions.add()
            motor.name = pos
            motor.position = positions[pos]
        return self.generate_message(request)

    def generate_message(self, message):
        """[summary]

        Args:
            message ([type]): [description]

        Returns:
            [type]: [description]
        """
        return message.ByteSize().to_bytes(4, byteorder='big', signed=False)+message.SerializeToString()

    def message_from_file(self, path):
        """[summary]

        Args:
            path ([type]): [description]

        Returns:
            [type]: [description]
        """
        return self.generate_message(self.build_request_from_file(path))

    def get_answer_size(self, content_size):
        size = int.from_bytes(content_size, byteorder='big', signed=False)
        return size

    def add_initial_request(self, sensor_name, sensor_time):
        if self.init_request is None:
            self.init_request = messages_pb2.ActuatorRequests()
        sensor = self.init_request.sensor_time_steps.add()
        sensor.name = sensor_name
        sensor.timeStep = sensor_time

    def build_initial_request(self):
        return self.generate_message(self.init_request)

    def parse_answer_message(self, data):
        """[summary]

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
        """[summary]

        Args:
            message ([type]): [description]

        Returns:
            dict: dict with keys of names sensors
        """
        parse_message = {}
        parse_message.update({"time": {"unix time": message.real_time, "sim time": message.time}})
        #if message.time % 100 == 0:
        #    print(f"message time={message.time}")
        for sensor in message.accelerometers:
            parse_message.update({sensor.name: {"position": [
                sensor.value.X, sensor.value.Y, sensor.value.Z], "time": message.time}})
        for sensor in message.cameras:
            parse_message.update({sensor.name: {"width": sensor.width, "height": sensor.height,
                                                "quality": sensor.quality, "image": sensor.image, 
                                                "time": message.time}})
        for sensor in message.position_sensors:
            parse_message.update(
                {sensor.name: {"position": sensor.value, "time": message.time}})
        for sensor in message.gyros:
            parse_message.update({sensor.name: {"position": [
                                 sensor.value.X, sensor.value.Y, sensor.value.Z], "time": message.time}})
        for sensor in message.gps:
            parse_message.update(
                {sensor.name: {"position": [sensor.value.X, sensor.value.Y], "time": message.time}})
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
                # if sensor.name == "BALL":
                #     parse_message.update(
                #         {sensor.name: {"position": [sensor.course, sensor.distance], "time": message.time}})
                # else:
                #     parse_message.update(
                #     {sensor.name: {"position": [sensor.course, sensor.distance], "time": message.time}})
        for sensor in message.imu:
            parse_message.update(
                {sensor.name: {"position": [sensor.angles.roll, sensor.angles.pitch,
                sensor.angles.yaw], "time": message.time}})
        for sensor in message.messages:
            parse_message.update(
                {sensor.name: {"message_type": sensor.message_type, "text": sensor.text, "time": message.time}})
        return parse_message
