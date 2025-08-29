#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorOutputs, VehicleOdometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import csv
import os
from math import atan2, asin, pi

def quaternion_to_euler(q):
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) <= 1:
        pitch = asin(sinp)
    else:
        pitch = pi / 2 * (1 if sinp > 0 else -1)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class SyncedLogger(Node):
    def __init__(self):
        super().__init__('synced_logger')

        # 노드 시작 시점의 시스템 시간(ns)를 기록 → 상대 타임스탬프 기준
        self.start_ns = self.get_clock().now().nanoseconds

        self.motor_msg = None
        self.odom_msg = None

        # 센서 토픽용 QoS: BEST_EFFORT
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # 실제 ESC 출력
        self.create_subscription(
            ActuatorOutputs,
            '/fmu/out/actuator_outputs',
            self.motor_cb,
            sensor_qos
        )
        # 차량 위치·속도·자세
        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_cb,
            sensor_qos
        )

        desktop = os.path.expanduser('~/Desktop')
        os.makedirs(desktop, exist_ok=True)
        csv_path = os.path.join(desktop, 'synced_log.csv')
        self.csv_file = open(csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)

        header = ['t_rel_ms'] + [f'motor{i}' for i in range(8)] + [
            'x','y','z',
            'vx','vy','vz',
            'rollrate','pitchrate','yawrate',
            'roll','pitch','yaw'
        ]
        self.writer.writerow(header)

        self.get_logger().info(f"Logging to {csv_path} (relative to node start)")

    def motor_cb(self, msg: ActuatorOutputs):
        self.motor_msg = msg

    def odom_cb(self, msg: VehicleOdometry):
        self.odom_msg = msg
        self.try_log()

    def try_log(self):
        if self.motor_msg is None or self.odom_msg is None:
            return

        # 현재 시스템 시간(ns)
        now_ns = self.get_clock().now().nanoseconds
        # 노드 시작 시점 대비 경과 시간(ms)
        t_rel_ms = (now_ns - self.start_ns) // 1_000_000

        motors = list(self.motor_msg.output[:8])
        self.motor_msg = None

        pos = list(self.odom_msg.position)
        vel = list(self.odom_msg.velocity)
        ang_vel = list(self.odom_msg.angular_velocity)
        roll, pitch, yaw = quaternion_to_euler(self.odom_msg.q)

        row = [t_rel_ms] + motors + pos + vel + ang_vel + [roll, pitch, yaw]
        self.writer.writerow(row)
        self.get_logger().info(f"[{t_rel_ms} ms] motors: {motors}")

        self.odom_msg = None

def main(args=None):
    rclpy.init(args=args)
    node = SyncedLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
