# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


from .control_algorithm import normalize_angle
import numpy as np


class PathGenerator:

    def __init__(self, s_x, s_y, s_yaw, g_x, g_y, g_yaw, is_backward=False):

        self.is_backward = is_backward
        if is_backward:
            self.s_yaw = normalize_angle(s_yaw + np.pi)
            self.g_yaw = normalize_angle(g_yaw + np.pi)
        else:
            self.s_yaw = s_yaw
            self.g_yaw = g_yaw
        self.s_x = s_x
        self.s_y = s_y
        self.g_x = g_x
        self.g_y = g_y
        # 경로생성 알고리즘 선택
        # self.plan_path = self.plan_simple_path
        self.plan_path = self.plan_dubins_path

    def plan_simple_path(self):
        s_x = self.s_x
        g_x = self.g_x
        s_y = self.s_y
        g_y = self.g_y

        dist = np.sqrt(pow(g_x-s_x, 2) + pow(g_y-s_y, 2))
        count = int(dist * 10) + 1  # 0.1m 간격으로 목표점 인터폴레이션
        map_xs = np.linspace(s_x, g_x, count)
        map_ys = np.linspace(s_y, g_y, count)
        map_yaws = np.arctan2(np.gradient(map_ys), np.gradient(map_xs))

        return map_xs, map_ys, map_yaws

    def plan_dubins_path(self):
        from .dubins_path import plan_dubins_path
        from .control_algorithm import MINIMUM_TURNING_RADIUS
        start_x = self.s_x
        start_y = self.s_y
        start_yaw = self.s_yaw

        end_x = self.g_x
        end_y = self.g_y
        end_yaw = self.g_yaw

        curvature = 1 / (MINIMUM_TURNING_RADIUS * 1.1)

        path_x, path_y, path_yaw, mode, lengths = \
            plan_dubins_path(start_x, start_y, start_yaw,
                             end_x, end_y, end_yaw,
                             curvature, 0.02)

        return path_x, path_y, path_yaw
