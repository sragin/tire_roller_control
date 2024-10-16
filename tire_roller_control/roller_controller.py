# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from action_msgs.msg import GoalStatus
from datetime import datetime
from geometry_msgs.msg import Twist
from math import dist
from msg_gps_interface.msg import GPSMsg
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from roller_interfaces.action import MoveToPosition
from roller_interfaces.msg import RollerStatus
from std_msgs.msg import Int8
from std_msgs.msg import String
import time

from .control_algorithm import BASE_COORDINATES
from .control_algorithm import MAX_STEER_LIMIT, GNSSTOWHEEL
from .control_algorithm import stanley_control
import roller_control.velocity_profile as velocity_profile
CONTROL_PERIOD = 0.1


class RollerController(Node):

    def __init__(self):
        super().__init__('roller_controller')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')
        qos_profile = QoSProfile(depth=10)
        self.rollermotioncmd_subscriber = self.create_subscription(
            String,
            'roller_motion_cmd',
            self.recieve_motioncmd,
            qos_profile
        )
        self.rollerstatus_subscriber = self.create_subscription(
            RollerStatus,
            'roller_status',
            self.recieve_rollerstatus,
            qos_profile)
        self.gps_msg_subscriber = self.create_subscription(
            GPSMsg,
            'gps_msg',
            self.recv_gpsmsg,
            qos_profile
        )
        self.radar_msg_subscriber = self.create_subscription(
            Int8,
            'radar_msg',
            self.recv_radarmsg,
            qos_profile
        )
        self._action_server = ActionServer(
            self,
            MoveToPosition,
            'move_to',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        self.control_timer = None
        self.velocity_profiler = None
        self.is_cancel_requested = False
        self.is_estop = False
        self.is_stop = False
        self.cmd_vel_msg = None

        self.basepoint = BASE_COORDINATES
        self.path_json = None
        self.map_xs = None
        self.map_ys = None
        self.map_yaws = None
        self.cmd_vel = None
        self.goal_check_error = 0.05

        self.roller_status = RollerStatus()
        self.gps_quality = 0
        self.gps_speed = 0.0
        self.radar_status = Int8()

        self.count = 0
        self.log_display_cnt = 50

    def recieve_motioncmd(self, msg: String):
        if msg.data == 'E-STOP':
            self.get_logger().warn(f'Recieved: {msg}')
            self.is_stop = False
        elif msg.data == 'STOP':
            self.get_logger().warn(f'Recieved: {msg}')
            self.is_stop = True

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Executing goal')
        self.get_logger().info(
            f'Goal info = xs:{self.map_xs[0] :.3f} xe:{self.map_xs[-1] :.3f} '
            f'ys:{self.map_ys[0] :.3f} ye:{self.map_ys[-1] :.3f} '
            f'vel: {self.cmd_vel[0]}'
        )
        feedback_msg = MoveToPosition.Feedback()
        self.velocity_profiler = velocity_profile.VelocityProfiler(self.cmd_vel[0])
        self.is_cancel_requested = False
        self.control_timer = self.create_timer(CONTROL_PERIOD, self.control)

        while self.control_timer is not None:
            if goal_handle.is_cancel_requested:
                self.is_cancel_requested = True
            time.sleep(0.05)

        if self.is_stop:
            # 감속정지 루프 실행
            # 타이머로 뺄 경우 모션이 끝나서 다음모션 실행가능하게되지만
            # 일단 정지할 때 까지 다음모션 실행하지 않도록 for 루프로 구현
            self.cmd_vel_msg.angular.z = 0.0
            vel = self.cmd_vel_msg.linear.x
            dec_time = 30  # 3초
            for i in range(dec_time-1, -1, -1):
                self.cmd_vel_msg.linear.x = vel * i / dec_time
                self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                time.sleep(0.1)
        else:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)

        result = MoveToPosition.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.result = False
            self.get_logger().info('Goal has been canceled')
        else:
            goal_handle.succeed()
            result.result = True
        return result

    def goal_callback(self, goal: MoveToPosition.Goal):
        self.get_logger().info('Received goal')
        self.map_xs = [p.x for p in goal.path_pose]
        self.map_ys = [p.y for p in goal.path_pose]
        self.map_yaws = [p.theta for p in goal.path_pose]
        self.cmd_vel = [v.linear.x for v in goal.path_cmd_vel]
        self.get_logger().info(f'{self.map_xs} {self.map_ys} {self.map_yaws} {self.cmd_vel}')
        if len(self.map_xs) == len(self.map_ys) == len(self.map_yaws) == len(self.cmd_vel):
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def control(self):
        """
        제어 알고리즘 실행구문.
        * 전진시 롤러 드럼을 기준으로 제어
        * 후진시 바디를 기준으로 제어
        -X방향(후진주행방향)을 +X방향으로 변환
        왼손좌표계사용 (좌회전:음수, 우회전:양수)
        """
        vel = self.cmd_vel[0]
        theta = self.roller_status.pose.theta
        if self.gps_quality != 4:
            self.get_logger().fatal(f"GPS failed. Quality is {self.gps_quality}")

        if vel < 0:
            # BX992좌표를 기준으로 뒷바퀴좌표를 계산
            theta_ = theta + np.pi
            x = self.roller_status.body_pose.x + GNSSTOWHEEL * np.cos(theta_)
            y = self.roller_status.body_pose.y + GNSSTOWHEEL * np.sin(theta_)
            steer_angle = self.roller_status.steer_angle
            self.get_logger().info(f'theta:{theta_} x:{self.roller_status.body_pose.x} {x} y:{self.roller_status.body_pose.y} {y}')
        else:
            x = self.roller_status.pose.x
            y = self.roller_status.pose.y
            steer_angle = self.roller_status.steer_angle
            theta_ = theta + steer_angle

        steer_, yaw_, cte_, min_dist_, min_index_ =\
            stanley_control(x=x, y=y, yaw=theta_, v=vel,
                            map_xs=self.map_xs, map_ys=self.map_ys, map_yaws=self.map_yaws)
        steer_cmd = np.clip(steer_, -MAX_STEER_LIMIT, MAX_STEER_LIMIT)

        self.cmd_vel_msg = Twist()
        start_p = (self.map_xs[0], self.map_ys[0])
        end_p = (self.map_xs[-1], self.map_ys[-1])
        cur_p = (x, y)
        dist_moved = dist(start_p, cur_p)
        dist_togo = dist(cur_p, end_p)
        self.velocity_profiler.set_maxvel(abs(self.cmd_vel[min_index_]))
        vel_cmd = self.velocity_profiler.get_simple_trapezoidal_profile_velocity(dist_moved, dist_togo)
        if vel < 0:
            vel_cmd = -vel_cmd
        self.cmd_vel_msg.linear.x = vel_cmd
        self.cmd_vel_msg.angular.z = steer_cmd

        # 안전디바이스 처리
        if self.radar_status.data == 2 or self.radar_status.data == 3:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0

        now = datetime.now()
        self.get_logger().info(
            f'{now.strftime("%Y-%m-%d %H:%M:%S")} '
            f'Controller = steer_:{steer_ :.3f}, yaw_:{yaw_ :.3f}, cte_:{cte_ :.3f}, '
            f'min_dist_:{min_dist_ :.3f} idx:{min_index_}\n'
            f'Position = xi:{self.map_xs[min_index_] :.3f} yi:{self.map_ys[min_index_] :.3f} '
            f'yaw_i(deg):{self.map_yaws[min_index_] * 180 / np.pi :.3f} '
            f'drum x:{self.roller_status.pose.x :.3f} y:{self.roller_status.pose.y :.3f} '
            f'yaw:{(self.roller_status.pose.theta + steer_angle) * 180 / np.pi} '
            f'body x:{self.roller_status.body_pose.x :.3f} y:{self.roller_status.body_pose.y :.3f} '
            f'yaw:{self.roller_status.pose.theta * 180 / np.pi}\n'
            f'Roller Status = steer angle(deg):{steer_angle * 180 / np.pi :.3f} '
            f'steer_cmd:{steer_cmd * 180 / np.pi :.3f} '
            f'cmd_vel:{vel_cmd :.2f} cur_vel:{self.gps_speed}\n'
            f'dist_moved: {dist_moved :.3f}, dist_togo:{dist_togo :.3f} '
            f'ERR:{y - self.map_ys[min_index_] :.3f} Radar:{self.radar_status.data}'
        )

        done = self.check_goal(self.map_xs, self.map_ys, x, y, self.goal_check_error)
        cancel_requested = self.is_cancel_requested
        if done or cancel_requested:
            self.control_timer.cancel()
            self.control_timer = None
            self.get_logger().info('Motion is done')
        else:
            self.cmd_vel_publisher.publish(self.cmd_vel_msg)

    def check_goal(self, map_xs, map_ys, x, y, error):
        x1 = map_xs[-1]
        y1 = map_ys[-1]

        # 거리가 error 보다 작으면 무조건 종료
        dist_togo = dist((x1, y1), (x, y))
        if dist_togo < error:
            # print('Distance to GOAL is less than error')
            return True

        """
        시작점(x)을 종료점(x1) 기준으로 90도 회전한 점을 x2라고 정의
        x1에서 x2 방향으로 이어지는 직선 왼쪽영역을 종료조건으로 하여
        현재위치가 왼쪽에 있는지 오른쪽에 있는지 벡터의 외적을 이용해서 계산한다
        """
        x2_trans1 = map_xs[0] - x1
        y2_trans1 = map_ys[0] - y1
        theta = np.pi/2
        x2_rot = x2_trans1 * np.cos(theta) - y2_trans1 * np.sin(theta)
        y2_rot = x2_trans1 * np.sin(theta) + y2_trans1 * np.cos(theta)
        x2 = x2_rot + x1
        y2 = y2_rot + y1

        cross = (x2 - x1)*(y - y1) - (x - x1) * (y2 - y1)
        # print(f'[{x1:.2f}, {y1:.2f}] [{x2:.2f}, {y2:.2f}] [{x:.2f}, {y:.2f}] {cross:.2f}')
        return cross >= 0

    def recieve_rollerstatus(self, msg: RollerStatus):
        self.roller_status = msg

    def recv_gpsmsg(self, msg: GPSMsg):
        self.gps_quality = msg.quality
        self.gps_speed = msg.speed

    def recv_radarmsg(self, msg: Int8):
        self.radar_status = msg

def main(args=None):
    rclpy.init(args=args)

    roller_controller = RollerController()
    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(roller_controller, executor=executor)
    roller_controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
