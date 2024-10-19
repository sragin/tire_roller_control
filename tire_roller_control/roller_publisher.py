# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


from msg_gps_interface.msg import GPSMsg
from msg_gps_interface.msg import GPSMsgAtt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile
from roller_interfaces.msg import RollerStatus
from std_msgs.msg import Float32

from tire_roller_control.control_algorithm import BASE_COORDINATES


class RollerPublisher(Node):

    def __init__(self):
        super().__init__('roller_publisher')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')
        qos_profile = QoSProfile(depth=10)

        self.gps_msg_subscriber = self.create_subscription(
            GPSMsg,
            'gps_msg',
            self.recv_gpsmsg,
            qos_profile_system_default
        )
        self.gps_msg_subscriber = self.create_subscription(
            GPSMsgAtt,
            'gps_att_msg',
            self.recv_gpsmsgatt,
            qos_profile_system_default
        )
        self.encoder_msg_subscriber = self.create_subscription(
            Float32,
            'encoder_msg',
            self.recv_encodermsg,
            qos_profile_sensor_data
        )
        self.roller_status_publisher = self.create_publisher(
            RollerStatus,
            'roller_status',
            qos_profile
        )

        self.timer_roller_geometry_msg = self.create_timer(1/20, self.publish_roller_geometry_msg)

        self.theta = 0.0  # radian. 바디의 헤딩각
        self.steer_angle = 0.0  # radian 스티어링 각도
        self.position = [0., 0.]  # position X(N), Y(E)
        self.response = [0, 0]
        self.speed = 0.0
        self.heading = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0

        # 소부연 테스트베드 원점. 임의로 정한값임. 수준점 측량 후 변경해줘야 함)
        self.basepoint = BASE_COORDINATES

        self.count = 0
        self.log_display_cnt = 10

    def recv_gpsmsg(self, msg: GPSMsg):
        # 로봇기준으로 X, Y 좌표 바꿈. 오른손 좌표계로 변환
        self.position[0] = msg.tm_y - self.basepoint[1]
        self.position[1] = msg.tm_x - self.basepoint[0]
        # 헤딩각과 조향각 방향 맞춤. East를 0도로 변경
        self.theta = (-msg.heading + 90 - 90) / 180 * np.pi
        self.theta = normalize_angle(self.theta)
        self.speed = msg.speed * 1000 / 3600  # km/h - m/s 단위변환
        self.heading = msg.heading
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt
        self.geo = msg.geo
        self.tm_x = msg.tm_x
        self.tm_y = msg.tm_y

    def recv_gpsmsgatt(self, msg: GPSMsgAtt):
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.heading = msg.heading

    def recv_encodermsg(self, msg: Float32):
        self.steer_angle = msg.data / 180 * np.pi

    def publish_roller_geometry_msg(self):
        # DRUM_LENGTH = 1.405  # 드럼이 BX992 기준점보다 앞서있는 거리. mm 단위
        msg = RollerStatus()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.steer_angle = self.steer_angle
        msg.pose.theta = self.theta
        msg.body_pose.theta = self.theta
        # 롤러좌표를 기준으로 드럼좌표를 계산하여 보낸다
        # msg.pose.x = self.position[0] + DRUM_LENGTH * np.cos(self.theta + self.steer_angle)
        # msg.pose.y = self.position[1] + DRUM_LENGTH * np.sin(self.theta + self.steer_angle)
        msg.pose.x = self.position[0]
        msg.pose.y = self.position[1]
        msg.body_pose.x = self.position[0]
        msg.body_pose.y = self.position[1]
        msg.speed = self.speed
        self.roller_status_publisher.publish(msg)
        self.get_logger().info(
            f'MODE: {self.response[0]}, STATUS:{self.response[1]},'
            f' STEER_ANGLE(deg):{self.steer_angle / np.pi * 180 :.2f}'
            f' HEAD body(deg):{self.theta / np.pi * 180 :.2f}, SPEED:{self.speed :.2f}'
            f' POS X:{self.position[0] :.3f}, Y:{self.position[1] :.3f}',
            throttle_duration_sec=0.99
        )


def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def main(args=None):
    rclpy.init(args=args)
    try:
        roller_publisher = RollerPublisher()
        rclpy.spin(roller_publisher)
    except KeyboardInterrupt:
        roller_publisher.get_logger().warn('Keyboard interrrupt (SIGINT)')
    finally:
        roller_publisher.destroy_node()


if __name__ == '__main__':
    main()
