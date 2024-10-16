# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import json
from msg_gps_interface.msg import GPSMsg
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile
from roller_interfaces.action import MoveToPosition
from statemachine import StateMachine, State
from statemachine.exceptions import TransitionNotAllowed
from std_msgs.msg import String
import time

from .control_algorithm import BASE_COORDINATES
from .path_generator import PathGenerator


class VibrationRollerStateMachine(StateMachine):
    idle = State(initial=True)
    preparing_goal = State()
    navigating = State()

    plan_path = (
        idle.to(preparing_goal)
        | preparing_goal.to(preparing_goal)
    )
    go = preparing_goal.to(navigating)
    stop = (
        idle.to(idle)
        | navigating.to(idle)
    )
    estop = (
        idle.to(idle)
        | navigating.to(idle)
    )
    navigation_done = navigating.to(preparing_goal)
    task_done = preparing_goal.to(idle)

    def __init__(self, nav):
        self.navigator :Navigator = nav
        super(VibrationRollerStateMachine, self).__init__()

    def on_enter_idle(self):
        self.navigator.get_logger().warn('idle state')

    def on_enter_preparing_goal(self):
        self.navigator.get_logger().warn('preparing_goal state')
        if self.navigator.auto_repeat:
            if self.navigator.plan_path():
                self.go()
            else:
                self.navigator.load_pathfile(self.navigator.filenamecmd)
                self.plan_path()
        elif self.navigator.auto_task:
            if self.navigator.plan_path():
                self.go()
            else:
                self.navigator.get_logger().warn('No more path left')
                self.task_done()
        else:
            if self.navigator.plan_path():
                self.navigator.get_logger().info('Path planning has been done')
            else:
                self.navigator.get_logger().warn('No more path left')
                self.task_done()

    def on_enter_navigating(self):
        self.navigator.get_logger().warn('navigating state')
        self.navigator.send_goal()

    def on_stop(self):
        self.navigator.get_logger().warn('stopping machine action')
        if self.navigator.goal_handle is not None:
            future = self.navigator.goal_handle.cancel_goal_async()
            future.add_done_callback(self.navigator.cancel_done)
        return

    def on_estop(self):
        self.navigator.get_logger().warn('E-Stop')
        if self.navigator.goal_handle is not None:
            future = self.navigator.goal_handle.cancel_goal_async()
            future.add_done_callback(self.navigator.cancel_done)
        return


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')
        qos_profile = QoSProfile(depth=10)
        self.rollermotioncmd_subscriber = self.create_subscription(
            String,
            'roller_motion_cmd',
            self.recieve_motioncmd,
            qos_profile
        )
        self.gps_msg_subscriber = self.create_subscription(
            GPSMsg,
            'gps_msg',
            self.recv_gpsmsg,
            qos_profile
        )

        self.sm = VibrationRollerStateMachine(self)
        self._action_client = ActionClient(self, MoveToPosition, 'move_to')

        self.basepoint = BASE_COORDINATES
        self.planning_index = 0
        self.auto_repeat = False
        self.auto_task = False
        self.path_json = None
        self.filename = None
        self.goal_handle = None
        self.gps_quality = 0
        self.filenamecmd = ''

    def recieve_motioncmd(self, msg: String):
        self.get_logger().info(f'{msg}')
        try:
            if msg.data =='E-STOP':
                self.sm.estop()
            elif msg.data == 'STOP' or msg.data == 'MANUAL':
                self.sm.stop()
            elif 'PATHFILE' in msg.data:
                self.load_pathfile(msg.data)
            elif msg.data == 'PLAN PATH':
                self.auto_task = False
                self.sm.plan_path()
            elif msg.data == 'START MOTION':
                if self.gps_quality != 4:
                    self.get_logger().error(f"Can't start. GPS quality is {self.gps_quality}")
                    return
                self.auto_task = False
                self.sm.go()
            elif msg.data == 'START TASK':
                if self.gps_quality != 4:
                    self.get_logger().error(f"Can't start. GPS quality is {self.gps_quality}")
                    return
                self.auto_task = True
                self.sm.go()
            elif msg.data == 'REPEAT OFF':
                self.auto_repeat = False
            elif msg.data == 'REPEAT ON':
                self.auto_repeat = True
        except TransitionNotAllowed as e:
            self.get_logger().warn(f'{e}')

    def recv_gpsmsg(self, msg: GPSMsg):
        self.gps_quality = msg.quality

    def load_pathfile(self, filenamecmd=''):
        json_string = ''
        if filenamecmd != '':
            _, json_string = filenamecmd.split(':', maxsplit=1)
        if json_string == '':
            return
        self.filenamecmd = filenamecmd
        # with open(self.filename, 'r') as pathfile:
        self.path_json = json.loads(json_string)
        # self.get_logger().info(f'Path file \'{self.filename.split("/")[-1]}\' has been loaded')
        self.get_logger().info(f'{self.path_json}')
        self.path_tm_x = self.path_json['tm_x']
        self.path_tm_y = self.path_json['tm_y']
        self.path_vel = self.path_json['vel']
        self.path_heading = self.path_json['heading']
        self.path_dir = self.path_json['dir']
        self.planning_index = 0

    def plan_path(self):
        if self.path_json is None:
            self.get_logger().warn('Select path file first')
            return False
        if self.planning_index >= len(self.path_tm_x) - 1:
            self.get_logger().warn('Planning done')
            return False

        self.map_xs, self.map_ys, self.map_yaws, self.cmd_vel = [], [], [], []
        while True:
            i = self.planning_index
            s_x = self.path_tm_y[i] - self.basepoint[1]
            s_y = self.path_tm_x[i] - self.basepoint[0]
            s_yaw = self.path_heading[i]
            ref_v = self.path_vel[i]
            is_backward = (self.path_dir[i] == 3)
            g_x = self.path_tm_y[i+1] - self.basepoint[1]
            g_y = self.path_tm_x[i+1] - self.basepoint[0]
            g_yaw = self.path_heading[i+1]
            print(f'{s_x :.3f},{s_y :.3f} {g_x :.3f},{g_y :.3f}')

            p = PathGenerator(s_x=s_x, s_y=s_y, s_yaw=s_yaw,
                            g_x=g_x, g_y=g_y, g_yaw=g_yaw,
                            is_backward=is_backward)
            xs, ys, yaws = p.plan_path()
            self.map_xs.extend(xs)
            self.map_ys.extend(ys)
            self.map_yaws.extend(yaws)
            self.cmd_vel.extend([ref_v] * len(xs))

            self.planning_index = i + 1
            if self.path_dir[i] != self.path_dir[i+1]:
                break
            if self.planning_index >= len(self.path_tm_x) - 1:
                break

        for i in range(len(self.map_xs)):
            print(f'x:{self.map_xs[i] :.3f}, y:{self.map_ys[i] :.3f},'
                  f' theta(deg):{self.map_yaws[i]/np.pi*180 :.3f},'
                  f' vel:{self.cmd_vel[i] :.2f}'
            )
        self.get_logger().info(f'backward:{is_backward} point index:{self.planning_index}')
        self.get_logger().info('Path has been loaded.')
        return True

    def send_goal(self):
        self.get_logger().info('Sending goal')
        goal_msg = MoveToPosition.Goal()
        # goal_msg.path_pose = []
        for i in range(len(self.map_xs)):
            pose = Pose2D()
            pose.x = self.map_xs[i]
            pose.y = self.map_ys[i]
            pose.theta = self.map_yaws[i]
            goal_msg.path_pose.append(pose)
        # goal_msg.path_cmd_vel = []
        for v in self.cmd_vel:
            twist = Twist()
            twist.linear.x = v
            goal_msg.path_cmd_vel.append(twist)

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.goal_handle = goal_handle
        self.get_logger().info('Goal accepted')
        self._get_result_future: Future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result: MoveToPosition.Result = future.result().result
        if result.result:
            self.get_logger().info(f'Motion succeeded')
            try:
                self.sm.navigation_done()
            except TransitionNotAllowed as e:
                self.get_logger().warn(f'{e}')
        else:
            self.get_logger().info(f'Motion failed')

    def cancel_done(self, future: Future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Motion stopped')
            self.planning_index = len(self.path_tm_x)
        else:
            self.get_logger().info('Motion failed to cancel')
            # 있어서는 안되는 상태이나 만일 도달했을 경우 직접 base controller에 명령 송신해야할 듯


def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
