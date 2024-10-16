import numpy as np


class VelocityProfiler:
    def __init__(self, v_max):
        self.acc_time = 5.0  # sec
        self.dec_time = 5.0  # sec
        self.v_min = 0.1  # m/s
        self.current_velocity = 0
        self.v_max = abs(v_max)
        self.acceleration = self.v_max / self.acc_time
        self.deceleration = self.v_max / self.dec_time
        power_vmax = self.v_max ** 2
        self.acc_dist = 0.5 * power_vmax / self.acceleration
        self.dec_dist = 0.5 * power_vmax / self.deceleration

    def set_maxvel(self, v_max):
        self.v_max = abs(v_max)
        self.acceleration = self.v_max / self.acc_time
        self.deceleration = self.v_max / self.dec_time
        power_vmax = self.v_max ** 2
        self.acc_dist = 0.5 * power_vmax / self.acceleration
        self.dec_dist = 0.5 * power_vmax / self.deceleration

    def get_simple_trapezoidal_profile_velocity(self, distance_moved, distance_togo):
        # print(f'{distance_togo} {self.dec_dist} {self.current_velocity} {self.v_max} {self.acceleration} {self.deceleration} {self.v_min}')
        # if distance_moved < self.acc_dist:
        #     vel = self.current_velocity + self.acceleration * 0.1
        if distance_togo <= self.dec_dist:
            vel = np.sqrt(2 * self.deceleration * np.abs(distance_togo)) - 0.25
        elif self.current_velocity < self.v_max:
            vel = self.current_velocity + self.acceleration * 0.1
            if vel > self.v_max:
                vel = self.v_max
        elif self.current_velocity > self.v_max:
            vel = self.current_velocity - self.deceleration * 0.1
        else:
            vel = self.v_max

        if vel < self.v_min:
            vel = self.v_min
        self.current_velocity = vel
        return vel
