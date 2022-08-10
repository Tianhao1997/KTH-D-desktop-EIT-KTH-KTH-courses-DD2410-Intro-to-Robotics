from dubins import Car
from dubins import step
import math

# {Yunfan YANG}
# {yunfan@kth.se}

car = Car()


class Node:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


def solution(car):

    x0 = car.x0
    y0 = car.y0
    xt = car.xt
    yt = car.yt

    start = Node(x0, y0, 0)
    goal = Node(xt, yt, 0)

    controls = []
    times = [0]
    cost = 0

    q = [[start, controls, times, cost]]
    q_q = [(round(x0, 2), round(y0, 2))]
    s = []

    while len(q) != 0:

        q.sort(key=lambda x: x[3] + get_dist(goal, x[0]))
        current = q.pop(0)

        s.append((round(current[0].x, 2), round(
            current[0].y, 2)))

        if get_dist(current[0], goal) < 1.5:
            return current[1], current[2]

        for phi in [-math.pi / 4, 0, math.pi / 4]:
            new_x = current[0].x
            new_y = current[0].y
            new_theta = current[0].theta
            new_controls = list(current[1])
            new_times = list(current[2])

            for i in range(100 if phi == 0 else 157):
                new_x, new_y, new_theta = step(
                    car, new_x, new_y, new_theta, phi)

                new_controls.append(phi)
                new_times.append(new_times[-1] + 0.01)

                if not good_condition(car, new_x, new_y):
                    break

            new_node = Node(new_x, new_y, new_theta)
            new_cost = get_dist(new_node, current[0]) + current[3]

            if not good_condition(car, new_node.x, new_node.y):
                continue

            if (round(new_x, 2), round(new_y, 2)) in s:
                continue

            if (round(new_x, 2), round(new_y, 2)) not in q_q:
                q.append([new_node, new_controls, new_times, new_cost])
                q_q.append((round(new_x, 2), round(
                    new_y, 2)))
            else:
                for i in range(len(q)):
                    if q_q[i] == (round(new_x, 2), round(new_y, 2)):
                        if q[i][3] > new_cost:
                            q[i][0] = new_node
                            q[i][1] = list(new_controls)
                            q[i][2] = list(new_times)

        controls = list(current[1])
        times = list(current[2])
    return controls, times


def get_dist(p1, p2):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def good_condition(car, x, y):
    if (x < car.xlb or x > car.xub or y < car.ylb or y > car.yub):
        return False
    for obss in car.obs:
        d = math.sqrt((x - obss[0])**2 + (y - obss[1])**2)
        if d <= obss[2] + 0.2:
            return False
    return True
