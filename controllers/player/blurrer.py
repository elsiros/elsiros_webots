
import json
import random

class Blurrer():
    """Simulate localization and vision noize. 
    Params is placed in the blurrer.json file.
    Args:
        object_angle_noize (float, optional): Noize for angle in radians.
            Blurrer will uniformly random value from -object_angle_noize 
            to object_angle_noize and add it to the ground truth course. 
            Defaults to 0.
        object_distance_noize (float, optional): Noize for distance in 
            percents divided by 100. Blurrer will uniformly random value 
            from -object_distance_noize to object_distance_noize and multiply 
            difference of 1 and this value with ground truth distance. 
            Defaults to 0..
        observation_bonus (float, optional): Blurrer will increase the 
            consistency for every good observation (successfuly processed 
            image). Defaults to 0..
        step_cost (float, optional): Blurrer will decrease the consistency
            for every simulation step. Defaults to 0..
        constant_loc_noize (float, optional): Constant localization noize. 
            Defaults to 0..
        loc_noize_meters (float, optional): Multiplier for consistency, in 
            meters. Defaults to 0. Defaults to 0..
    """

    def __init__(self, object_angle_noize=0., object_distance_noize=0.,
                 observation_bonus=0., step_cost=0., 
                 constant_loc_noize=0., loc_noize_meters=0.):

        self.object_angle_noize = object_angle_noize
        self.object_distance_noize = object_distance_noize
        self.observation_bonus = observation_bonus
        self.step_cost = step_cost
        self.constant_loc_noize = constant_loc_noize
        self.loc_noize_meters = loc_noize_meters

        params = self.load_json("../player/blurrer.json")

        self.consistency = 1
        self.receiver = None
        #penalty = self.receiver.player_state.penalty

    def load_json(self, filename):
        with open(filename) as f:
            params = json.load(f)

        self.object_angle_noize = params["object_angle_noize"]
        self.object_distance_noize = params["object_distance_noize"]
        self.observation_bonus = params["observation_bonus"]
        self.step_cost = params["step_cost"]
        self.constant_loc_noize = params["constant_loc_noize"]
        self.loc_noize_meters = params["loc_noize_meters"]

    def course(self, angle):
        return angle + random.uniform(-self.object_angle_noize, self.object_angle_noize)

    def distance(self, distance):
        return distance * (1 + random.uniform(-self.object_distance_noize, self.object_distance_noize))

    def objects(self, course=course, distance=distance):
        return (self.course(course), self.distance(distance))

    def loc(self, x, y):
        return (self.coord(x), self.coord(y))

    def coord(self, p):
        random_factor = 1 - self.consistency
        return p + self.loc_noize_meters * random.uniform(-random_factor, random_factor)

    def step(self):
        self.update_consistency(-self.step_cost)

    def observation(self):
        self.update_consistency(self.observation_bonus)

    def update_consistency(self, value):
        tmp = self.consistency + value
        self.consistency = max(0, min(tmp, 1))