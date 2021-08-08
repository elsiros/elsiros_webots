import queue

from google.protobuf import text_format

import messages_pb2


class MessageManager():
    def __init__(self, init_buffer_size=4):
        self.size = init_buffer_size

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

    def parse_answer_message(self, data):
        message = messages_pb2.SensorMeasurements()
        message.ParseFromString(data)
        return self.parse_to_dict(message)

    def parse_to_dict(self, message):
        parse_message = {}
        # time stamp at which the measurements were performed expressed in [ms]
        parse_message.update({"time": message.time})
        # // real unix time stamp at which the measurements were performed in [ms]
        parse_message.update({"real_time": message.real_time})
        parse_message.update({"messages": message.messages})
        parse_message.update({"accelerometers": message.accelerometers})
        parse_message.update({"bumpers": message.bumpers})
        parse_message.update({"cameras": message.cameras})
        parse_message.update({"forces": message.forces})
        parse_message.update({"force3ds": message.force3ds})
        parse_message.update({"force6ds": message.force6ds})
        parse_message.update({"gyros": message.gyros})
        parse_message.update({"position_sensors": message.position_sensors})
        parse_message.update({"gps": message.gps})
        return parse_message
