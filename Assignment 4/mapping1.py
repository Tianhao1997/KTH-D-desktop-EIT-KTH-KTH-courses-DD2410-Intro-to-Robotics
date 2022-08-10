#!/usr/bin/env python3

"""
    # {Tianhao He}
    # {19971105-T533}
    # {tianhaoh@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs
import math
# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()


        """
        Fill in your solution here
        """

        # setup a list to store occupied grid
        occupied_grid = []

        for i in np.arange(len(scan.ranges)):
            range_value = scan.ranges[i]
            # ignore the out bounded range values
            if range_value <= scan.range_min or range_value >= scan.range_max:
                continue
            # get the total yaw of the current state
            total_yaw = robot_yaw + scan.angle_min + scan.angle_increment * i
            # get x and y values --- convert to integer
            temp_x = int(((cos(total_yaw) * scan.ranges[i]) + (pose.pose.position.x - origin.position.x)) / resolution)
            temp_y = int(((sin(total_yaw) * scan.ranges[i]) + (pose.pose.position.y - origin.position.y)) / resolution)
            # store the occupied x y values
            occupied_grid.append((temp_x, temp_y))

            # get non-occupied grid info using supplied raytrace(self, start, end) function
            current_start_point = [int((pose.pose.position.x - origin.position.x) / resolution), int((pose.pose.position.y - origin.position.y) / resolution)]
            non_occupied_grid = self.raytrace(current_start_point, occupied_grid[-1])
            
            # fill in free space using function add_to_map(self, grid_map, x, y, value)
            for j in non_occupied_grid:
                self.add_to_map(grid_map, j[0], j[1], self.free_space)
        # fill in occupied space using function add_to_map(self, grid_map, x, y, value)
        for k in occupied_grid:
            self.add_to_map(grid_map, k[0], k[1], self.occupied_space)


        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = min(item[0] for item in occupied_grid)
        # The minimum y index in 'grid_map' that has been updated
        update.y = min(item[1] for item in occupied_grid)
        # Maximum x index - minimum x index + 1
        update.width = max(item[0] for item in occupied_grid) - min(item[0] for item in occupied_grid) + 1 
        # Maximum y index - minimum y index + 1
        update.height = max(item[1] for item in occupied_grid) - min(item[1] for item in occupied_grid) + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []
        h = update.height
        w = update.width
        for l in np.arange(h):
            for m in np.arange(w):
                update.data.append(grid_map.__getitem__([l, m]))

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """
        radius = self.radius
        looping_range = radius * 2 
        for i in np.arange(grid_map.get_width()):
            for j in np.arange(grid_map.get_height()):
                if grid_map[i, j] == self.occupied_space:
                    x_corr_val = i - radius
                    y_corr_val = j - radius
                    for x in np.arange(looping_range):
                        for y in np.arange(looping_range):
                            final_x = int(x + x_corr_val)
                            final_y = int(y + y_corr_val)
                            if grid_map[final_x, final_y] != self.occupied_space:
                                if self.is_in_bounds(grid_map, final_x, final_y):
                                    if radius >= math.sqrt((x - i + x_corr_val)**2+(y - j + y_corr_val)**2):
                                        self.add_to_map(grid_map, final_x, final_y, self.c_space) 
        # Return the inflated map
        return grid_map
