import queue

from google.protobuf import text_format

import messages_pb2


class MessageManager():
    def __init__(self, init_buffer_size=4):
        self.size = init_buffer_size
        self.init_request = 0

    def get_size(self):
        return self.size

    def create_requests_message(self):
        return messages_pb2.ActuatorRequests()

    def create_answer_message(self):
        return messages_pb2.SensorMeasurements()

    def build_request_from_file(self, path):
        request = messages_pb2.ActuatorRequests()
        with open(path) as actuator_requests:
            text_format.Parse(actuator_requests.read(), request)
        return request

    def build_request_positions(self, positions):
        request = messages_pb2.ActuatorRequests()
        for pos in positions:
            motor = request.motor_positions.add()
            motor.name = pos
            motor.position = positions[pos]
        return self.generate_message(request)

    def generate_message(self, message):
        return message.ByteSize().to_bytes(4, byteorder='big', signed=False)+message.SerializeToString()

    def message_from_file(self, path):
        return self.generate_message(self.build_request_from_file(path))

    def get_answer_size(self, content_size):
        size = int.from_bytes(content_size, byteorder='big', signed=False)
        return size

    def add_initial_request(self, sensor_name, sensor_time):
        if self.init_request == 0:
            self.init_request = messages_pb2.ActuatorRequests()
        sensor = self.init_request.sensor_time_steps.add()
        sensor.name = sensor_name
        sensor.timeStep = sensor_time

    def build_initial_request(self):
        return self.generate_message(self.init_request)

    def parse_answer_message(self, data):
        message = messages_pb2.SensorMeasurements()
        message.ParseFromString(data)
        return self.parse_message(message)

    def parse_message(self, message):
        parse_message = {}
        #parse_message.update({"real_time": message.real_time})
        #parse_message.update({"messages": message.messages})
        #parse_message.update({"accelerometers": message.accelerometers})
        for sensor in message.accelerometers:
            parse_message.update({sensor.name: {"position": [
                                 sensor.value.X, sensor.value.Y, sensor.value.Z], "time": message.time}})
        for sensor in message.cameras:
            parse_message.update({sensor.name: {"width": sensor.width, "height": sensor.height,
                                 "quality": sensor.quality, "image": sensor.image, "time": message.time}})
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
                if sensor.name == "ball":
                    parse_message.update(
                        {sensor.name: {"position": [sensor.X, sensor.Y], "time": message.time}})
                elif sensor.name == "robot":
                    parse_message.update(
                    {sensor.name: {"position": [sensor.X, sensor.Y], "time": message.time}})
        
        for sensor in message.imu:
            parse_message.update(
                {sensor.name: {"position": [sensor.angles.pitch, sensor.angles.roll, sensor.angles.yaw], "time": message.time}})
        return parse_message
