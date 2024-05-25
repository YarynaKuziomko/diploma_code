import pygame
import math
import numpy as np

GRAVITY = 0.08
MASS = 1
ARM = 25
THRUSTER_AMPLITUDE = 0.04
THRUSTER_MEAN = 0.04
DIFF_AMPLITUDE = 0.003

Q1 = math.pi / 4
Q2 = 3 * math.pi / 4
Q3 = 5 * math.pi / 4
Q4 = 7 * math.pi / 4

SENSOR_LEN = 100
SENSOR_COUNT = 50
DRONE_W = 68

ANG_UP = math.pi / 2
ANG_DOWN = 3 * math.pi / 2


# distance between two points
def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)


class Drone:
    def __init__(self, start_pos, width):
        self.m2p = 3779.52
        self.player_width = width

        self.x_position = start_pos[0]
        self.y_position = start_pos[1]

        self.x_acceleration = 0
        self.y_acceleration = 0

        self.thruster_left = 0
        self.thruster_right = 0

        self.angle = 1.5707963267949

        self.angular_speed = 0
        self.angular_acceleration = 0

        self.min_obs_dist = 50

        self.is_up = False
        self.is_down = False
        self.is_left = False
        self.is_right = False
        self.is_changed = False
        self.is_direction_selected = False
        self.is_min_dist_reached = False
        self.direction = "right"

        self.drone_done = False

        self.path = []

    def avoid_obstacles(self, point_cloud, no_obstacle_endpoints, target_x, target_y, drone_position):

        closest_obs = None
        dist = np.inf

        print("an up ", self.angles_up())
        print("an down ", self.angles_down())

        print("no obs", len(no_obstacle_endpoints))
        print("obs ", len(point_cloud))

        counts_obs = {
            'q1_q2': sum(Q1 <= item[2] < Q2 for item in point_cloud),
            'q2_q3': sum(Q2 <= item[2] < Q3 for item in point_cloud),
            'q3_q4': sum(Q3 <= item[2] < Q4 for item in point_cloud),
            'q4_q1': sum((item[2] < Q1 or item[2] >= Q4) for item in point_cloud)
        }

        min_key, min_value = min(counts_obs.items(), key=lambda x: x[1])

        print("min key ", min_key)
        print("min value ", min_value)
        print(counts_obs)

        counts_no_obs = {
            'q1_q2': sum(Q1 <= item[2] < Q2 for item in no_obstacle_endpoints),
            'q2_q3': sum(Q2 <= item[2] < Q3 for item in no_obstacle_endpoints),
            'q3_q4': sum(Q3 <= item[2] < Q4 for item in no_obstacle_endpoints),
            'q4_q1': sum((item[2] < Q1 or item[2] >= Q4) for item in no_obstacle_endpoints)
        }

        max_key, max_value = max(counts_no_obs.items(), key=lambda x: x[1])

        print("max key ", max_key)
        print("max value ", max_value)
        print(counts_no_obs)

        print("is_up ", self.is_up)
        print("is_dow ", self.is_down)
        print("is_l ", self.is_left)
        print("is_r ", self.is_right)

        if len(point_cloud) > 1:
            for point in point_cloud:
                if dist > distance([self.x_position, self.y_position], point[:2]):
                    dist = distance([self.x_position, self.y_position], point[:2])
                    closest_obs = (point, dist)

            print("closest ", closest_obs)

        print("thruster_left ", self.thruster_left)
        print("thruster_right ", self.thruster_right)

        print("up sensor counts", self.can_go_up(no_obstacle_endpoints))
        print("down sensor counts", self.can_go_down(no_obstacle_endpoints))
        print("no obs mass ", no_obstacle_endpoints)

        if drone_position == "up":
            print("drone pos ", drone_position)
            if target_y - 10 < self.y_position < target_y + 10:
                if self.is_changed:
                    self.thruster_left = 0
                    self.thruster_right = 0
                    self.is_changed = False
                if target_x - 10 < self.x_position < target_x + 10:
                    print("drone done")
                    self.drone_done = True
                elif target_x > self.x_position:
                    self.move_right()
                elif target_x < self.x_position:
                    self.move_left()
            else:
                if closest_obs[1] > self.min_obs_dist and self.is_q1(
                        closest_obs[0][2]) and not self.is_direction_selected:
                    print("cl obs dist", closest_obs[1])
                    if not self.is_changed:
                        self.thruster_left = 0
                        self.thruster_right = 0
                    self.is_changed = True
                    self.move_up()
                elif not self.is_direction_selected and (
                        self.is_q2(closest_obs[0][2]) or self.is_q4(closest_obs[0][2])):
                    print("cl obs dist", closest_obs[1])
                    if not self.is_changed:
                        self.thruster_left = 0
                        self.thruster_right = 0
                    self.is_changed = True
                    self.move_up()
                else:
                    if not self.is_direction_selected:
                        print("direction not selected")
                        if counts_obs['q4_q1'] < counts_obs['q2_q3']:
                            self.direction = "right"
                            self.is_direction_selected = True
                        else:
                            self.direction = "left"
                            self.is_direction_selected = True
                    else:
                        print("direction selected")
                        if self.can_go_up(no_obstacle_endpoints) >= 6:
                            if not self.is_changed:
                                self.thruster_left = 0
                                self.thruster_right = 0
                            self.is_changed = True
                            self.move_up()
                        else:
                            if self.direction == "right":
                                if self.is_changed:
                                    self.thruster_left = 0
                                    self.thruster_right = 0
                                    self.is_changed = False
                                if closest_obs[1] > self.min_obs_dist and self.is_q4(closest_obs[0][2]):
                                    self.is_direction_selected = False
                                else:
                                    if counts_no_obs['q1_q2'] == 0 and closest_obs[
                                        1] < self.min_obs_dist and self.is_q4(
                                        closest_obs[0][2]):
                                        self.direction = "left"
                                    else:
                                        self.move_right()
                            elif self.direction == "left":
                                if self.is_changed:
                                    self.thruster_left = 0
                                    self.thruster_right = 0
                                    self.is_changed = False
                                if closest_obs[1] > self.min_obs_dist and self.is_q2(closest_obs[0][2]):
                                    self.is_direction_selected = False
                                else:
                                    if counts_no_obs['q1_q2'] == 0 and closest_obs[
                                        1] < self.min_obs_dist and self.is_q2(
                                        closest_obs[0][2]):
                                        self.direction = "right"
                                    else:
                                        self.move_left()
        else:
            print("drone pos ", drone_position)
            if target_y - 10 < self.y_position < target_y + 10:
                if self.is_changed:
                    self.thruster_left = 0
                    self.thruster_right = 0
                    self.is_changed = False
                if target_x - 10 < self.x_position < target_x + 10:
                    print("drone done")
                elif target_x > self.x_position:
                    self.move_right()
                elif target_x < self.x_position:
                    self.move_left()
            else:
                if closest_obs[1] > self.min_obs_dist and self.is_q3(
                        closest_obs[0][2]) and not self.is_direction_selected:
                    print("cl obs dist", closest_obs[1])
                    if not self.is_changed:
                        self.thruster_left = 0
                        self.thruster_right = 0
                    self.is_changed = True
                    self.move_down()
                elif not self.is_direction_selected and (
                        self.is_q2(closest_obs[0][2]) or self.is_q4(closest_obs[0][2])):
                    print("cl obs dist", closest_obs[1])
                    if not self.is_changed:
                        self.thruster_left = 0
                        self.thruster_right = 0
                    self.is_changed = True
                    self.move_down()
                else:
                    if not self.is_direction_selected:
                        print("direction not selected")
                        if counts_obs['q4_q1'] < counts_obs['q2_q3']:
                            self.direction = "right"
                            self.is_direction_selected = True
                        else:
                            self.direction = "left"
                            self.is_direction_selected = True
                    else:
                        print("direction selected")
                        if self.can_go_down(no_obstacle_endpoints) >= 6:
                            if not self.is_changed:
                                self.thruster_left = 0
                                self.thruster_right = 0
                            self.is_changed = True
                            self.move_down()
                        else:
                            if self.direction == "right":
                                if self.is_changed:
                                    self.thruster_left = 0
                                    self.thruster_right = 0
                                    self.is_changed = False
                                if closest_obs[1] > self.min_obs_dist and self.is_q4(closest_obs[0][2]):
                                    self.is_direction_selected = False
                                else:
                                    if counts_no_obs['q1_q2'] == 0 and closest_obs[
                                        1] < self.min_obs_dist and self.is_q4(closest_obs[0][2]):
                                        self.direction = "left"
                                    else:
                                        self.move_right()
                            elif self.direction == "left":
                                if self.is_changed:
                                    self.thruster_left = 0
                                    self.thruster_right = 0
                                    self.is_changed = False
                                if closest_obs[1] > self.min_obs_dist and self.is_q2(closest_obs[0][2]):
                                    self.is_direction_selected = False
                                else:
                                    if counts_no_obs['q1_q2'] == 0 and closest_obs[
                                        1] < self.min_obs_dist and self.is_q2(closest_obs[0][2]):
                                        self.direction = "right"
                                    else:
                                        self.move_left()

    def can_go_up(self, counts_no_obs):
        return sum(1 for i in counts_no_obs if self.angles_up()[0] < i[2] < self.angles_up()[1])

    def can_go_down(self, counts_no_obs):
        return sum(1 for i in counts_no_obs if self.angles_down()[0] < i[2] < self.angles_down()[1])

    def is_q1(self, angle):
        return Q1 < angle < Q2

    def is_q2(self, angle):
        return Q2 < angle < Q3

    def is_q3(self, angle):
        return Q3 < angle < Q4

    def is_q4(self, angle):
        return Q1 < angle and angle > Q4

    def angles_up(self):
        arc_length = DRONE_W + 10

        start_angle = math.pi / 2 - (int((arc_length / SENSOR_LEN) / (2 * math.pi / SENSOR_COUNT)) // 2) * (
                2 * math.pi / SENSOR_COUNT)
        end_angle = math.pi / 2 + (int((arc_length / SENSOR_LEN) / (2 * math.pi / SENSOR_COUNT)) // 2) * (
                2 * math.pi / SENSOR_COUNT)

        return start_angle, end_angle

    def angles_down(self):
        arc_length = DRONE_W + 10

        start_angle = 3 * math.pi / 2 - (int((arc_length / SENSOR_LEN) / (2 * math.pi / SENSOR_COUNT)) // 2) * (
                2 * math.pi / SENSOR_COUNT)
        end_angle = 3 * math.pi / 2 + (int((arc_length / SENSOR_LEN) / (2 * math.pi / SENSOR_COUNT)) // 2) * (
                2 * math.pi / SENSOR_COUNT)

        return start_angle, end_angle

    def len_arc(self):
        return SENSOR_LEN * (2 * math.pi / SENSOR_COUNT)

    def move_down(self):
        print("down")
        self.thruster_left += DIFF_AMPLITUDE
        self.thruster_right += DIFF_AMPLITUDE
        self.is_down = True
        self.is_up = False
        self.is_left = False
        self.is_right = False

    def move_up(self):
        print("up")
        self.thruster_left += DIFF_AMPLITUDE
        self.thruster_right += DIFF_AMPLITUDE
        self.is_up = True
        self.is_down = False
        self.is_left = False
        self.is_right = False

    def move_left(self):
        print("left")
        self.thruster_right = 0
        self.thruster_left -= DIFF_AMPLITUDE
        self.is_left = True
        self.is_up = False
        self.is_down = False
        self.is_right = False

    def move_right(self):
        print("right")
        self.thruster_left = 0
        self.thruster_right -= DIFF_AMPLITUDE
        self.is_right = True
        self.is_up = False
        self.is_down = False
        self.is_left = False

    def kinematics(self):

        self.path.append([self.x_position, self.y_position])

        if self.is_down:
            self.y_position -= (
                -(self.thruster_left + self.thruster_right)/2)
        elif self.is_up:
            self.y_position += (
                -(self.thruster_left + self.thruster_right)/2)
        elif self.is_left:
            self.x_position -= -(self.thruster_left + self.thruster_right)/2
        elif self.is_right:
            self.x_position += -(self.thruster_left + self.thruster_right)/2

        print("x ", self.x_position)
        print("y ", self.y_position)


class Graphics:
    def __init__(self, dimensions, drone_img_path, map_img_path):
        pygame.init()

        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yel = (255, 255, 0)
        self.grey = (61, 61, 61)

        self.robot = pygame.image.load(drone_img_path)
        self.map_img = pygame.image.load(map_img_path)

        self.height, self.width = dimensions

        pygame.display.set_caption("obs avoid")
        self.map = pygame.display.set_mode((self.height, self.width))
        self.map.blit(self.map_img, (0, 0))

    def draw_drone(self, x, y, angle):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(angle), 1)
        rect = rotated.get_rect(center=(x, y))
        self.map.blit(rotated, rect)

    def graw_sensor_data(self, point_cloud, no_obstacle_endpoints):
        for point in point_cloud:
            pygame.draw.circle(self.map, self.red, point[:2], 3, 0)
        for point in no_obstacle_endpoints:
            pygame.draw.circle(self.map, self.green, point[:2], 3, 0)

    def draw_target(self, target_x, target_y):
        pygame.draw.circle(self.map, self.blue, (target_x, target_y), 20, 0)

    def draw_text(self, text):
        self.map.blit(pygame.font.Font(None, 36).render(text, True, self.red),
                      (self.width // 2, self.height // 2))

    def draw_path(self, path):
        for point in path:
            pygame.draw.circle(self.map, self.grey, (point[0], point[1]), 1, 0)


class Ultrasonic:

    def __init__(self, sensor_range, map):
        self.sensor_range = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.map = map

    def sense_obstacles(self, x, y, angle):
        obstacles_endpoints = []
        no_obstacle_endpoints = []

        x1, y1 = x, y
        start_angle = 2 * angle - self.sensor_range[1]
        finish_angle = 2 * angle + self.sensor_range[1]

        for ang in np.linspace(start_angle, finish_angle, SENSOR_COUNT, False):
            x2 = x1 + self.sensor_range[0] * math.cos(ang)
            y2 = y1 - self.sensor_range[0] * math.sin(ang)
            obstacle_detected = False
            for i in range(0, SENSOR_LEN):
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.map_width and 0 < y < self.map_height:
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 255, 255))
                    if color == (0, 0, 0):
                        obstacles_endpoints.append([x, y, abs(ang)])
                        obstacle_detected = True
                        break

            if not obstacle_detected:
                no_obstacle_endpoints.append([x, y, abs(ang)])

        return obstacles_endpoints, no_obstacle_endpoints
