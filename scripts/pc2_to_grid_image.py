#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
import time
import cv2
import tf
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import laser_geometry.laser_geometry as lg
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan


class Pc2ToGrid():
    def __init__(self):
        # get parameters
        self.save_path = rospy.get_param("~save_path")
        lrs_pc2_topic = rospy.get_param("~lrs_pc2_topic")
        self.lrs_frame = rospy.get_param("~lrs_frame")
        self.grid_size = rospy.get_param("~grid_size")
        self.map_size = [rospy.get_param("~x_min"), rospy.get_param("~x_max"), rospy.get_param("~y_min"), rospy.get_param("~y_max")]

        # initialize
        self.sub_obstacle_pc2 = rospy.Subscriber(lrs_pc2_topic, PointCloud2, self.lrs_pc2_handler)
        self.tf_listener = tf.TransformListener()
        self.save_file_index = 0
        self.obstacle_arr = None
        self.non_obstacle_arr = None
        self.lrs_pc2_processing = False

        self.rate = rospy.Rate(10)
        self.grid_x_width = None
        self.grid_y_width = None
        self.grid_x_min = None
        self.grid_x_max = None
        self.grid_y_min = None
        self.grid_y_max = None

    def lrs_pc2_handler(self, _msg):
        if not self.lrs_pc2_processing:
            self.lrs_pc2_processing = True

            # set grid size
            if self.grid_x_width is None:
                self.set_grid(self.grid_size, self.map_size)

            self.obstacle_arr = self.generate_arr(_msg)
            self.non_obstacle_arr = self.get_non_obstacle_arr(_msg)
            self.generate_grid(self.obstacle_arr, self.non_obstacle_arr, self.grid_size)

            # wait
            self.rate.sleep()

            self.lrs_pc2_processing = False

    def get_non_obstacle_arr(self, pc2_msg):
        point_generator = pc2.read_points(pc2_msg)
        [xl, yl] = self.get_robot_tf()
        print("({}, {})".format(xl, yl))
        non_obstacle_points = [0] * 1000000
        all_points_i = 0
        for point in point_generator:
            # (xp, yp, zp) : obstacle point
            # (xl, yl, zl) : lrs (robot) position
            xp = point[0]
            yp = point[1]
            d = 0.05  # distance between points interpolated
            point_i = 0
            while True:
                dx = (xp - xl) * d / math.sqrt((xp - xl) ** 2 + (yp - yl) ** 2)
                dy = (yp - yl) * d / math.sqrt((xp - xl) ** 2 + (yp - yl) ** 2)
                x = xl + dx * point_i
                y = yl + dy * point_i
                non_obstacle_points[all_points_i] = [x, y]
                if (abs(xp - x) < 0.1) or (abs(yp - y) < 0.1):
                    break
                else:
                    point_i += 1
                    all_points_i += 1
        del non_obstacle_points[all_points_i + 1:]
        non_obstacle_arr = np.array(non_obstacle_points)
        print("COMPLETE : {}".format(all_points_i))
        return non_obstacle_arr

    def generate_arr(self, _pc2_msg):
        point_generator = pc2.read_points(_pc2_msg)
        points = []
        for point in point_generator:
            if not math.isnan(point[2]):
                points.append([point[0], point[1]])  # (x, y)
        arr = np.array(points)
        return arr

    def set_grid(self, _grid_size, _map_size):
        [x_min, x_max, y_min, y_max] = _map_size  # [m]
        self.grid_x_min = round(x_min / _grid_size)
        self.grid_x_max = round(x_max / _grid_size)
        self.grid_y_min = round(y_min / _grid_size)
        self.grid_y_max = round(y_max / _grid_size)

        self.grid_x_width = int(self.grid_x_max - self.grid_x_min + 1)
        self.grid_y_width = int(self.grid_y_max - self.grid_y_min + 1)

        print("grid_x_width : {}".format(self.grid_x_width))
        print("grid_y_width : {}".format(self.grid_y_width))

    def generate_grid(self, _obstacle_arr, non_obstacle_arr, _grid_size):
        grid_arr = np.zeros((self.grid_x_width, self.grid_y_width), dtype=np.int)

        # initialize all at 127 (center of 0 ~ 255)
        grid_arr = np.full_like(grid_arr, 127)

        obstacle_arr_round = np.round(_obstacle_arr / _grid_size)
        non_obstacle_arr_round = np.round(non_obstacle_arr / _grid_size)

        # obstacle points filled with black (0)
        for point_i in range(obstacle_arr_round.shape[0]):
            x_obstacle = obstacle_arr_round[point_i][0]
            y_obstacle = obstacle_arr_round[point_i][1]
            grid_arr[int(x_obstacle - self.grid_x_min), int(y_obstacle - self.grid_y_min)] = 0

        # non obstacle points filled with white (255)
        for point_i in range(non_obstacle_arr_round.shape[0]):
            x_non_obstacle = non_obstacle_arr_round[point_i][0]
            y_non_obstacle = non_obstacle_arr_round[point_i][1]
            grid_arr[int(x_non_obstacle - self.grid_x_min), int(y_non_obstacle - self.grid_y_min)] = 255

        cv2.imwrite(self.save_path + "/" + str(self.save_file_index) + ".jpg", grid_arr)
        print("save_file_index : {}".format(self.save_file_index))
        self.save_file_index += 1

    def get_robot_tf(self):
        while not rospy.is_shutdown():
            try:
                (translation, rotation) = self.tf_listener.lookupTransform("/map", self.lrs_frame, rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        [x, y, z] = translation
        [roll, pitch, yaw] = tf.transformations.euler_from_quaternion(rotation)
        return x, y


def main():
    rospy.init_node("pc2_to_grid")

    print("===== PointCloud2 to Grid Convertor ======================")
    pc2_to_grid = Pc2ToGrid()

    rospy.spin()


if __name__ == '__main__':
    main()
