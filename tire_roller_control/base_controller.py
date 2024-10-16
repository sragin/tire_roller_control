# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


from ament_index_python import get_package_share_directory
from can_msgs.msg import Frame
import cantools
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from roller_interfaces.msg import RollerStatus
from std_msgs.msg import String
from teleop_msgs.msg import RollerTeleop

CONTROL_PERIOD = 0.1


class BaseController(Node):

    def __init__(self):
        super().__init__('base_controller')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')

        self.candb_controller = cantools.db.load_file(
            get_package_share_directory('roller_control') + '/Controller_230823.dbc')
        self.can_msg_control = self.candb_controller.get_message_by_name('CONTROLLER_COMM')
        self.candb_commandsv = cantools.db.load_file(
            get_package_share_directory('roller_control') + '/ToSupervisor_210430.dbc')
        self.can_msg_commandsv = self.candb_commandsv.get_message_by_name('Command_SV')

        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Frame, 'to_can_bus', qos_profile)
        self.cmdvel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.recieve_cmdvel,
            qos_profile
        )
        self.rollerstatus_subscriber = self.create_subscription(
            RollerStatus,
            'roller_status',
            self.recieve_rollerstatus,
            qos_profile
        )
        self.rollermotioncmd_subscriber = self.create_subscription(
            String,
            'roller_motion_cmd',
            self.recieve_motioncmd,
            qos_profile)
        self.rollerteleopcmd_subscriber = self.create_subscription(
            RollerTeleop,
            'teleop_cmd',
            self.recv_teleop_cmd,
            qos_profile
        )
        self.create_timer(CONTROL_PERIOD, self.velocity_controller)
        self.create_timer(CONTROL_PERIOD, self.steering_controller)
        self.create_timer(CONTROL_PERIOD, self.send_cancommand)

        self.cmd_drv_vel = 0.
        self.cmd_steer_pos = 0.
        self.out_velocity = 0.
        self.out_steering = 0.
        self.vel_pid = PID()
        self.vel_pid.SetTunnings(500, 0.0, 0.0)
        self.vel_pid.SetOutputLimits(1000.0, -1000.0)
        self.steer_pid = PID()
        self.steer_pid.SetTunnings(500, 0., 0.)
        self.steer_pid.SetOutputLimits(100.0, -100.0)
        self.steer_filter = LowPassFilter(1, CONTROL_PERIOD)
        self.vel_filter = LowPassFilter(0.2, CONTROL_PERIOD)
        self.mode = 0
        self.horn = 0
        self.vibration_mode = 0
        self.vibration_control = 0
        self.travel_mode = 0
        self.teleop_cmd = RollerTeleop()

        self.roller_status = RollerStatus()
        self.last_steer = 0.

        self.count = 0
        self.log_display_cnt = 10

    def recieve_cmdvel(self, msg: Twist):
        self.cmd_drv_vel = msg.linear.x
        self.cmd_steer_pos = msg.angular.z

    def recieve_rollerstatus(self, msg: RollerStatus):
        self.roller_status = msg

    def velocity_controller(self):
        vel_ = self.roller_status.speed
        vel = int(self.roller_status.speed * 100) / 100
        # 후진일경우 음수를 붙여준다
        if self.out_velocity < 0:
            vel *= -1
        out = self.vel_pid.Compute(self.cmd_drv_vel, vel)
        out = self.cmd_drv_vel / 1.25 * 1000
        out = np.clip(out, -1000, 1000)
        if self.cmd_drv_vel == 0.0:
            out = 0.0
            self.vel_pid.Reset()
        self.out_velocity = out
        self.get_logger().info(
            f'Velocity cmd: {self.cmd_drv_vel} vel: {vel :.3f} '
            f'raw: {vel_ :.5f} out: {out : .1f}'
        )

    def steering_controller(self):
        cur_steer = self.roller_status.steer_angle
        if self.cmd_steer_pos == 0.0:
            out = 0.0
            self.steer_pid.Reset()
        else:
            out = self.steer_pid.Compute(self.cmd_steer_pos, cur_steer)
        self.out_steering = out
        self.get_logger().info(
            f'Steer cmd: {self.cmd_steer_pos/np.pi*180 :.2f}'
            f' cur: {cur_steer/np.pi*180 :.3f} valve out: {out :.2f}'
        )

    def recieve_motioncmd(self, msg: String):
        self.get_logger().info(f'{msg}')
        if msg.data == 'MANUAL':
            self.mode = 0
        elif msg.data == 'AUTO':
            self.mode = 1
        elif msg.data == 'REMOTE':
            self.mode = 2
        elif 'HORN' in msg.data:
            if msg.data[len('HORN '):] == 'ON':
                self.horn = 1
            else:
                self.horn = 0
        elif 'VIBRATION BTN' in msg.data:
            if msg.data == 'VIBRATION BTN PRESSED':
                self.vibration_control = 1
            else:
                self.vibration_control = 0
        elif 'VIBRATION' in msg.data:
            if msg.data[len('VIBRATION '):] == 'HIGH':
                self.vibration_mode = 2
            elif msg.data[len('VIBRATION '):] == 'LOW':
                self.vibration_mode = 1
            else:
                self.vibration_mode = 0
        elif 'TRAVEL' in msg.data:
            if msg.data[len('TRAVEL '):] == 'RAMP':
                self.travel_mode = 1
            elif msg.data[len('TRAVEL '):] == 'F UPHILL':
                self.travel_mode = 2
            elif msg.data[len('TRAVEL '):] == 'R UPHILL':
                self.travel_mode = 3
            elif msg.data[len('TRAVEL '):] == 'RABBIT':
                self.travel_mode = 4
            else:
                self.travel_mode = 0

    def recv_teleop_cmd(self, msg: RollerTeleop):
        self.teleop_cmd = msg

    def send_cancommand(self):
        if self.mode == 1 or self.mode == 2:
            mode_ = 1
        else:
            mode_ = 0
        commandsv = self.can_msg_commandsv.encode(
            {'MODE': mode_, 'AUTO_DRIVE': 0, 'STOP_CMD': 0})
        cmdsv_msg = Frame()
        cmdsv_msg.id = self.can_msg_commandsv.frame_id
        cmdsv_msg.dlc = self.can_msg_commandsv.length
        cmdsv_msg.data[:cmdsv_msg.dlc] = list(commandsv)
        self.publisher_.publish(cmdsv_msg)

        steer_left_ = 0
        steer_right_ = 0
        out_velocity_ = 0
        if self.mode == 2:
            steer_left_ = self.teleop_cmd.steer_left
            steer_right_ = self.teleop_cmd.steer_right
            out_velocity_ = self.teleop_cmd.drive
        elif self.mode == 1:
            out_velocity_ = self.out_velocity
            steer_angle = self.roller_status.steer_angle
            if self.out_steering > 0 and steer_angle < 30.0:
                steer_left_ = self.out_steering
                steer_right_ = 0
            elif self.out_steering < 0 and steer_angle > -30.0:
                steer_right_ = -self.out_steering
                steer_left_ = 0

        data = self.can_msg_control.encode({
            'LEFT_DUTY_CONTROL': steer_left_,
            'RIGHT_DUTY_CONTROL': steer_right_,
            'AUTO_SPD_CONTROL': out_velocity_,
            'VIB_MODE': self.vibration_mode,
            'VIB_CONTROL': self.vibration_control,
            'DRV_MODE': self.travel_mode,
            'HORN': self.horn,
            'RESERVED': 0})
        control_msg = Frame()
        control_msg.id = self.can_msg_control.frame_id
        control_msg.dlc = self.can_msg_control.length
        control_msg.data[:control_msg.dlc] = list(data)
        self.publisher_.publish(control_msg)

        if self.count == self.log_display_cnt:
            self.get_logger().warning(
                f'Controller Command id: {control_msg.id} data: {control_msg.data}'
            )
            self.get_logger().warning(
                f'Supervisor Command id: {cmdsv_msg.id} data: {cmdsv_msg.data}'
            )
            # self.get_logger().info(f'Velocity cmd: {self.cmd_drv_vel} out: {self.out_velocity}')
            # self.get_logger().info(f'Steering cmd: {self.cmd_steer_vel}'
            #   ' out: {self.out_steering}')
            self.count = 0
        self.count += 1


class PID():

    def __init__(self):
        self.lastInput = 0.
        self.errSum = 0.
        self.lastErr = 0.
        self.ITerm = 0.

        self.kp = 0.1
        self.ki = 0.
        self.kd = 0.

        self.outMax = 1.
        self.outMin = 0.

    def Compute(self, Setpoint, Input):
        # Compute all the working error variables
        error = Setpoint - Input
        self.ITerm += (self.ki * error)
        if self.ITerm > self.outMax:
            self.ITerm = self.outMax
        elif self.ITerm < self.outMin:
            self.ITerm = self.outMin
        dInput = (Input - self.lastInput)

        # Compute PID Output
        Output = self.kp * error + self.ITerm - self.kd * dInput
        if Output > self.outMax:
            Output = self.outMax
        elif Output < self.outMin:
            Output = self.outMin

        self.lastErr = error
        return Output

    def SetTunnings(self, Kp, Ki, Kd):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd

    def SetOutputLimits(self, Max, Min=0.0):
        if Min > Max:
            return
        self.outMin = Min
        self.outMax = Max

    def Reset(self):
        self.lastInput = 0.
        self.errSum = 0.
        self.lastErr = 0.
        self.ITerm = 0.


class LowPassFilter(object):

    def __init__(self, cut_off_freqency, ts):
        # cut_off_freqency: 차단 주파수
        # ts: 주기

        self.ts = ts
        self.cut_off_freqency = cut_off_freqency
        self.tau = self.get_tau()

        self.prev_data = 0.

    def get_tau(self):
        return 1 / (2 * np.pi * self.cut_off_freqency)

    def lowpass_filter(self, data):
        val = (self.ts * data + self.tau * self.prev_data) / (self.tau + self.ts)
        self.prev_data = val
        return val


def main(args=None):
    rclpy.init(args=args)
    try:
        base_controller = BaseController()
        rclpy.spin(base_controller)
    except KeyboardInterrupt:
        base_controller.get_logger().info('Keyboard interrrupt (SIGINT)')
    finally:
        base_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
