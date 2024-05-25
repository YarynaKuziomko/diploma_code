import math
import pygame

from Drone import Graphics, Drone, Ultrasonic

#MAP_DIMENSIONS = (800, 1000)
MAP_DIMENSIONS = (1300, 900)

gfx = Graphics(MAP_DIMENSIONS, "media/drone.png", "media/map2.png")

start = (500, 400)
drone = Drone(start, 0.01 * 3779.52)

sensor_range = 180, math.radians(180)
ultra_sonic = Ultrasonic(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()

step = 1

target_x = 1250
target_y = 850

running = True

if target_y < drone.y_position:
    drone_position = "up"
else:
    drone_position = "down"

while running:
    print("\n----------------------- step ", step, " ---------------------------------\n")
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    gfx.map.blit(gfx.map_img, (0, 0))
    drone.kinematics()
    gfx.draw_target(target_x, target_y)
    gfx.draw_drone(drone.x_position, drone.y_position, drone.angle)
    point_cloud, no_obstacle_endpoints = ultra_sonic.sense_obstacles(drone.x_position, drone.y_position, drone.angle)
    drone.avoid_obstacles(point_cloud, no_obstacle_endpoints, target_x, target_y, drone_position)
    gfx.graw_sensor_data(point_cloud, no_obstacle_endpoints)
    gfx.draw_path(drone.path)

    pygame.display.update()

    step += 1
