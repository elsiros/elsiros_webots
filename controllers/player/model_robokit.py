import math

class Model():
    def __init__(self, robot, blurrer):
        self.robot = robot
        self.max_visible_area = 4
        self.min_visible_area = 0
        self.fov_x = math.radians(45) / 2
        self.fov_y = math.radians(60) / 2
        self.robot_height = 0.413
        self.blurrer = blurrer

    def check_robot_stand(self):
        servos_sum = 0
        for angle in self.robot.last_message.values():
            servos_sum += angle

        # print(f"Sum angle: {servos_sum}")
        if servos_sum >= 0.01:
            return False
        else:
            return True

    def check_object_in_frame(self, distance, course):
        right_yaw_visible_area = -self.robot.last_head_yaw - self.fov_y
        left_yaw_visible_area = -self.robot.last_head_yaw + self.fov_y

        if -self.robot.last_head_pitch < self.fov_x:
            top_distance_visible_area = self.max_visible_area
        else:
            top_distance_visible_area = self.robot_height * \
                math.tan(math.pi/2 +
                         self.robot.last_head_pitch +
                         self.fov_x)

        if -self.robot.last_head_pitch > math.pi/2 - self.fov_x:
            bottom_distance_visible_area = self.min_visible_area
        else:
            bottom_distance_visible_area = self.robot_height * \
                math.tan(math.pi/2 +
                         self.robot.last_head_pitch -
                         self.fov_x)

        print(f"right_yaw_visible_area: {right_yaw_visible_area}, left_yaw_visible_area: {left_yaw_visible_area}, \
                 top_distance_visible_area: {top_distance_visible_area}, bottom_distance_visible_area: {bottom_distance_visible_area} \
                 distance: {distance}, course: {course}, self.fov_y: {self.fov_y}, self.fov_x: {self.fov_x},\
                 self.robot_yaw: {self.robot.last_head_yaw} self.robot_pitch: {self.robot.last_head_pitch}")
        if ((bottom_distance_visible_area < distance < top_distance_visible_area) and
                (right_yaw_visible_area < course < left_yaw_visible_area)):
            return True
        else:
            return False

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

    def get_distance_course(self, x, y, robot_gps, robot_imu):
        robot_pos = robot_gps["position"]
        distance = self.dist(robot_pos, (x, y))
        robot_orientation = robot_imu["position"]
        angle = -math.atan2(robot_pos[1] - y, -(robot_pos[0] - x)) - robot_orientation[2]
        angle = self.norm_yaw(angle)
        return (distance, angle)

    def proccess_data(self, x, y, robot_gps, robot_imu):
        self.blurrer.step()
        if not self.check_robot_stand():
            # print("WARNING: Robot in not standing")
            return []
        res = self.get_distance_course(x, y, robot_gps, robot_imu)
        if not res:
            # print("WARNING: Imu or gps not available")
            return []
        distance, angle = res

        self.blurrer.observation()
        if self.check_object_in_frame(distance, angle):
            return [angle, distance]
        else:
            # print("WARNING: Ball is not in the frame")
            return []