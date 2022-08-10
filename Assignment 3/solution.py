#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Tianhao He}
# {19971105-T533}
# {tianhaoh@kth.se}


from dubins import *
from math import sqrt, pi
import numpy as np

car = Car()
# making point easily
class Point:
    def __init__(point, x, y, theta):
        point.x = x
        point.y = y
        point.theta = theta

def check_out_of_boundary(car, x, y):
    bound_min_x, bound_max_x = car.xlb, car.xub
    bound_min_y, bound_max_y = car.ylb, car.yub
    if x <= bound_min_x or x >= bound_max_x or y <= bound_min_y or y >= bound_max_y:
        return 1
    return 0

def check_runin_obs(car, x, y):
    obs_list = car.obs
    for i in obs_list:
        if sqrt((x - i[0])**2 + (y - i[1])**2) - i[2] <= 0.1:
            return 1
    return 0

def solution(car):

    # store the values from car
    x0, y0 = car.x0, car.y0
    xt, yt = car.xt, car.yt
    theta = 0
    target = Point(xt, yt, theta)

    controls = [0]
    times = [0, 0.1]
    dist = 0
    stop = 0
    pass_record = []
    phi_list = [-pi/4, 0, pi/4]
    start_info = [[Point(x0, y0, theta), controls, times, dist]]
    point_pass = [(np.round(x0, 2), np.round(y0, 2))]

    while True:

        start_info = sorted(start_info, key=lambda dist: dist[3] + sqrt((target.x - dist[0].x)**2 + (target.y - dist[0].y)**2))
        game_info = start_info.pop(0)

        pass_record.append((np.round(game_info[0].x, 2), np.round(game_info[0].y, 2)))

        if sqrt((game_info[0].x - target.x)**2 + (game_info[0].y - target.y) ** 2) < 1.5:
            return game_info[1], game_info[2]

        for phi in phi_list:
            temp_x = game_info[0].x
            temp_y = game_info[0].y
            temp_theta = game_info[0].theta
            temp_controls = list(game_info[1])
            temp_times = list(game_info[2])
            if phi == 0:
                for i in np.arange(100):
                    temp_x, temp_y, temp_theta = step(car, temp_x, temp_y, temp_theta, phi)

                    temp_controls.append(phi)
                    temp_times.append(temp_times[-1] + 0.01)
                    stop = 0
                    stop = check_out_of_boundary(car, temp_x, temp_y)
                    stop = check_runin_obs(car, temp_x, temp_y)
                    if stop == 1:
                        break
            else:
                for i in np.arange(2 * 3.14 * 100 / 4):
                    temp_x, temp_y, temp_theta = step(car, temp_x, temp_y, temp_theta, phi)

                    temp_controls.append(phi)
                    temp_times.append(temp_times[-1] + 0.01)
                    stop = 0
                    stop = check_out_of_boundary(car, temp_x, temp_y)
                    stop = check_runin_obs(car, temp_x, temp_y)
                    if stop == 1:
                        break


            pointn = Point(temp_x, temp_y, temp_theta)
            temp_cost = game_info[3] + sqrt((pointn.x - game_info[0].x)**2 + (pointn.y - game_info[0].y)**2)

            if check_out_of_boundary(car, pointn.x, pointn.y) == 1 or check_runin_obs(car, pointn.x, pointn.y) == 1 or (np.round(temp_x, 2), np.round(temp_y, 2)) in pass_record:
                continue

            if (np.round(temp_x, 2), np.round(temp_y, 2)) not in point_pass:
                start_info.append([pointn, temp_controls, temp_times, temp_cost])
                point_pass.append((np.round(temp_x, 2), np.round(temp_y, 2)))
            else:
                for i in np.arange(len(start_info)):
                    if point_pass[i] == (np.round(temp_x, 2), np.round(temp_y, 2)):
                        if start_info[i][3] > temp_cost:

                            start_info[i][2] = list(temp_times)
                            start_info[i][0] = pointn
                            start_info[i][1] = list(temp_controls)

        controls.append(game_info[1])
        times.append(game_info[2])
    return controls, times
