import math
import logging

class Model():
    def __init__(self, blurrer):
        self.max_visible_area = 4
        self.min_visible_area = 0
        self.fov_x = math.radians(45) / 2
        self.fov_y = math.radians(60) / 2
        self.robot_height = 0.413
        self.blurrer = blurrer
        self.robot_gps = {}
        self.robot_imu = {}
        self.last_message = {}
        self.last_head_pitch = 0
        self.last_head_yaw = 0

    def check_robot_stand(self):
        servos_sum = 0
        for angle in self.last_message.values():
            servos_sum += angle

        # print(f"Sum angle: {servos_sum}")
        if servos_sum >= 0.01:
            return False
        else:
            return True

    def check_object_in_frame(self, distance, course):
        right_yaw_visible_area = -self.last_head_yaw - self.fov_y
        left_yaw_visible_area = -self.last_head_yaw + self.fov_y

        if -self.last_head_pitch < self.fov_x:
            top_distance_visible_area = self.max_visible_area
        else:
            top_distance_visible_area = self.robot_height * \
                math.tan(math.pi/2 +
                         self.last_head_pitch +
                         self.fov_x)

        if -self.last_head_pitch > math.pi/2 - self.fov_x:
            bottom_distance_visible_area = self.min_visible_area
        else:
            bottom_distance_visible_area = self.robot_height * \
                math.tan(math.pi/2 +
                         self.last_head_pitch -
                         self.fov_x)

        ball_in_dist = (bottom_distance_visible_area < distance < top_distance_visible_area)
        ball_in_yaw = (right_yaw_visible_area < course < left_yaw_visible_area)
        return ball_in_dist and ball_in_yaw


    @staticmethod
    def dist(p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    @staticmethod
    def norm_yaw(yaw):
        yaw %= 2 * math.pi
        if yaw > math.pi:
            yaw -= 2 * math.pi
        if yaw < -math.pi:
            yaw += 2 * math.pi
        return yaw

    def get_distance_course(self, x, y):
        robot_pos = self.robot_gps["position"]
        distance = self.dist(robot_pos, (x, y))
        robot_orientation = self.robot_imu["position"]
        angle = -math.atan2(robot_pos[1] - y, -(robot_pos[0] - x)) - robot_orientation[2]
        angle = self.norm_yaw(angle)
        return (distance, angle)

    def proccess_data(self, x, y):
        self.blurrer.step()
        if not self.check_robot_stand():
            logging.info("Robot in not standing")
            return []
        res = self.get_distance_course(x, y)
        if not res:
            logging.info("Imu or gps not available")
            return []
        distance, angle = res

        self.blurrer.observation()
        if self.check_object_in_frame(distance, angle):
            return self.blurrer.objects(course=angle, distance=distance)
        else:
            logging.info("Ball is not in the frame")
            return []

    def update_robot_state(self, gps, imu, last_message, last_head_pitch, last_head_yaw):
        self.robot_gps = gps
        self.robot_imu = imu
        self.last_message = last_message
        self.last_head_pitch = last_head_pitch
        self.last_head_yaw = last_head_yaw
