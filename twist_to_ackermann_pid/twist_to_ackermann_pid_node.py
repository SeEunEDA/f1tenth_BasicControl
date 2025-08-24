#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import SetParametersResult

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class TwistToAckermannPID(Node):
    """
    /cmd_vel (Twist) -> /drive (AckermannDriveStamped)
      - Speed: PID on /odom speed, with accel limiting and LPF
      - Steering: bicycle model (omega->delta) with rate limiting
    """

    def __init__(self) -> None:
        super().__init__('twist_to_ackermann_pid')

        # ---------------- Parameters ----------------
        # Topics & frame
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('frame_id', 'base_link')

        # Vehicle & limits
        self.declare_parameter('wheelbase', 0.33)              # [m]
        self.declare_parameter('max_speed', 2.5)               # [m/s]
        self.declare_parameter('max_steering_deg', 28.0)       # [deg]
        self.declare_parameter('rate_hz', 50.0)                # [Hz]
        self.declare_parameter('accel_limit', 2.0)             # [m/s^2]
        self.declare_parameter('max_steer_rate_deg_s', 240.0)  # [deg/s]

        # PID & filters
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.2)
        self.declare_parameter('kd', 0.04)
        self.declare_parameter('i_limit', 2.0)
        self.declare_parameter('vel_lpf_alpha', 0.2)  # 0..1 (higher = less smoothing)
        self.declare_parameter('stop_deadband', 0.05) # [m/s]

        # Load parameter values
        self.cmd_topic: str = self.get_parameter('cmd_vel_topic').value
        self.odom_topic: str = self.get_parameter('odom_topic').value
        self.drive_topic: str = self.get_parameter('drive_topic').value
        self.frame_id: str = self.get_parameter('frame_id').value

        self.L: float = float(self.get_parameter('wheelbase').value)
        self.vmax: float = float(self.get_parameter('max_speed').value)
        self.delta_max: float = math.radians(float(self.get_parameter('max_steering_deg').value))
        self.rate: float = float(self.get_parameter('rate_hz').value)
        self.accel_limit: float = float(self.get_parameter('accel_limit').value)
        self.max_steer_rate: float = math.radians(float(self.get_parameter('max_steer_rate_deg_s').value))

        self.kp: float = float(self.get_parameter('kp').value)
        self.ki: float = float(self.get_parameter('ki').value)
        self.kd: float = float(self.get_parameter('kd').value)
        self.i_limit: float = float(self.get_parameter('i_limit').value)
        self.alpha: float = float(self.get_parameter('vel_lpf_alpha').value)
        self.stop_db: float = float(self.get_parameter('stop_deadband').value)

        # Live tuning callback
        self.add_on_set_parameters_callback(self._on_param_change)

        # ---------------- State ----------------
        self.target_v: float = 0.0
        self.target_w: float = 0.0
        self.meas_v: float = 0.0
        self.v_filt: float = 0.0

        self.e_int: float = 0.0
        self.e_prev: Optional[float] = None
        self.t_prev: Optional[float] = None

        self.prev_cmd_v: float = 0.0
        self.prev_delta: float = 0.0

        # ---------------- I/O ----------------
        self.create_subscription(Twist, self.cmd_topic, self._cmd_cb, 10)
        # /odom as sensor QoS for timely updates
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, qos_profile_sensor_data)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        # Optional debug publishers
        self.pub_err  = self.create_publisher(Float32, 'pid_error', 3)
        self.pub_cmdv = self.create_publisher(Float32, 'cmd_v', 3)
        self.pub_measv= self.create_publisher(Float32, 'meas_v', 3)

        # Timer
        self.timer = self.create_timer(max(1e-3, 1.0 / self.rate), self._on_timer)

        self.get_logger().info(
            f'PID+Ackermann ready | L={self.L:.3f}m vmax={self.vmax:.2f}m/s '
            f'steer<=±{math.degrees(self.delta_max):.1f}deg rate={self.rate:.1f}Hz '
            f'PID(kp={self.kp}, ki={self.ki}, kd={self.kd}) accel_limit={self.accel_limit:.2f}m/s² '
            f'steer_rate<=±{math.degrees(self.max_steer_rate):.1f}deg/s'
        )

    # --------------- Callbacks ---------------
    def _on_param_change(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'kp': self.kp = float(p.value)
            elif p.name == 'ki': self.ki = float(p.value)
            elif p.name == 'kd': self.kd = float(p.value)
            elif p.name == 'max_speed': self.vmax = float(p.value)
            elif p.name == 'max_steering_deg': self.delta_max = math.radians(float(p.value))
            elif p.name == 'wheelbase': self.L = float(p.value)
            elif p.name == 'accel_limit': self.accel_limit = float(p.value)
            elif p.name == 'max_steer_rate_deg_s': self.max_steer_rate = math.radians(float(p.value))
            elif p.name == 'vel_lpf_alpha': self.alpha = float(p.value)
            elif p.name == 'stop_deadband': self.stop_db = float(p.value)
            elif p.name == 'cmd_vel_topic': self.cmd_topic = str(p.value)
            elif p.name == 'odom_topic': self.odom_topic = str(p.value)
            elif p.name == 'drive_topic': self.drive_topic = str(p.value)
            elif p.name == 'frame_id': self.frame_id = str(p.value)
        return SetParametersResult(successful=True)

    def _cmd_cb(self, msg: Twist) -> None:
        self.target_v = float(msg.linear.x)
        self.target_w = float(msg.angular.z)

    def _odom_cb(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        meas = math.hypot(vx, vy)
        # 1st-order low-pass filter
        self.v_filt = (1.0 - self.alpha) * self.v_filt + self.alpha * meas
        self.meas_v = meas
        # debug
        self.pub_measv.publish(Float32(data=float(self.meas_v)))

    # --------------- Control Loop ---------------
    def _on_timer(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t_prev is None:
            self.t_prev, self.e_prev = now, 0.0
            return
        dt = max(1e-4, now - self.t_prev)

        # ----- Speed PID -----
        e = self.target_v - self.v_filt
        if abs(self.target_v) < self.stop_db:
            # reset integrator near zero target to avoid creep
            self.e_int = 0.0

        de = (e - (self.e_prev if self.e_prev is not None else 0.0)) / dt
        u = self.kp * e + self.ki * self.e_int + self.kd * de  # desired speed (pre-limits)

        # Hard speed clamp
        u = clamp(u, -self.vmax, self.vmax)

        # Accel limiting (slew on speed command)
        dv_max = self.accel_limit * dt
        cmd_v = clamp(self.prev_cmd_v + clamp(u - self.prev_cmd_v, -dv_max, dv_max), -self.vmax, self.vmax)

        # Anti-windup: if saturated/limited, bleed integrator slightly
        if abs(u - cmd_v) > 1e-3:
            self.e_int *= 0.98
        else:
            self.e_int = clamp(self.e_int + e * dt, -self.i_limit, self.i_limit)

        # ----- Steering conversion (omega -> delta) -----
        # omega = v * tan(delta) / L  =>  delta = atan(L * omega / v)
        v_for = math.copysign(max(0.2, abs(cmd_v)), cmd_v)  # preserve sign, avoid div by small
        delta_raw = math.atan(self.L * self.target_w / v_for)
        delta_raw = clamp(delta_raw, -self.delta_max, self.delta_max)

        # Slew-rate limit on steering
        dmax = self.max_steer_rate * dt
        delta = clamp(self.prev_delta + clamp(delta_raw - self.prev_delta, -dmax, dmax),
                      -self.delta_max, self.delta_max)

        # ----- Publish -----
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.drive.speed = float(cmd_v)
        out.drive.steering_angle = float(delta)
        self.pub_drive.publish(out)

        # debug
        self.pub_err.publish(Float32(data=float(e)))
        self.pub_cmdv.publish(Float32(data=float(cmd_v)))

        # update state
        self.e_prev, self.t_prev = e, now
        self.prev_cmd_v, self.prev_delta = cmd_v, delta

    # Publish zero command a few times for safety
    def publish_zero(self, n: int = 3) -> None:
        z = AckermannDriveStamped()
        z.header.frame_id = self.frame_id
        z.drive.speed = 0.0
        z.drive.steering_angle = 0.0
        for _ in range(max(1, n)):
            z.header.stamp = self.get_clock().now().to_msg()
            self.pub_drive.publish(z)


def main() -> None:
    rclpy.init()
    node = TwistToAckermannPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info('Publishing zero command on shutdown...')
            node.publish_zero(3)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

