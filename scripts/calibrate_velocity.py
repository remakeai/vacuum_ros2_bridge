#!/usr/bin/env python3
"""
Velocity calibration helper for SangamIO-bridged vacuums (e.g. Proscenic M6 Pro).

Commands a steady LINEAR or ANGULAR velocity for a fixed duration, measures the
actual motion from /odom, and suggests the SangamIO velocity scale to use.

Run with the bridge up and the robot on the floor with clear space around it:

  ros2 run vacuum_ros2_bridge calibrate_velocity.py --linear 0.2 --duration 4 --current-scale 1726
  ros2 run vacuum_ros2_bridge calibrate_velocity.py --angular 0.6 --duration 4 --current-scale 523

Then set linear_velocity_scale / angular_velocity_scale in sangamio.toml to the
suggested value and restart SangamIO. linear and angular are independent.

Note: this measures via /odom, which itself depends on the bridge's
ticks-per-meter / wheel-base. For a ground-truth check, also measure the distance
(tape measure) or turn (protractor / count rotations) physically.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Calibrator(Node):
    def __init__(self):
        super().__init__('velocity_calibrator')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Best-effort sub is compatible with both reliable and best-effort /odom.
        self.create_subscription(Odometry, '/odom', self._odom_cb,
                                 qos_profile_sensor_data)
        self.x = 0.0
        self.y = 0.0
        self.cum_yaw = 0.0          # continuously-accumulated yaw (handles >180deg turns)
        self._last_yaw = None
        self.got_odom = False

    def _odom_cb(self, msg):
        p = msg.pose.pose
        self.x = p.position.x
        self.y = p.position.y
        yaw = yaw_from_quat(p.orientation)
        if self._last_yaw is not None:
            self.cum_yaw += wrap(yaw - self._last_yaw)
        self._last_yaw = yaw
        self.got_odom = True

    def cmd(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.pub.publish(t)

    def stop(self):
        for _ in range(5):
            self.cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)


def spin_for(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def main():
    ap = argparse.ArgumentParser(description="SangamIO velocity calibration helper")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--linear', type=float, metavar='M_S',
                   help='commanded linear speed in m/s, e.g. 0.2')
    g.add_argument('--angular', type=float, metavar='RAD_S',
                   help='commanded angular speed in rad/s, e.g. 0.6')
    ap.add_argument('--duration', type=float, default=4.0,
                    help='seconds to drive (default 4)')
    ap.add_argument('--current-scale', type=float, default=None,
                    help='current scale from sangamio.toml (to compute the new value)')
    args = ap.parse_args()

    linear_mode = args.linear is not None
    speed = args.linear if linear_mode else args.angular
    unit = 'm/s' if linear_mode else 'rad/s'

    rclpy.init()
    node = Calibrator()
    try:
        node.get_logger().info("Waiting for /odom (is the bridge running?)...")
        t0 = time.time()
        while not node.got_odom and time.time() - t0 < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.got_odom:
            node.get_logger().error("No /odom received. Aborting.")
            return 1

        node.get_logger().info(
            f"{'LINEAR' if linear_mode else 'ANGULAR'} test: commanding "
            f"{speed:+.3f} {unit} for {args.duration:.1f}s.")
        node.get_logger().warn("Make sure the robot has clear space! Starting in:")
        for n in (3, 2, 1):
            node.get_logger().info(f"  {n}...")
            spin_for(node, 1.0)

        x0, y0, yaw0 = node.x, node.y, node.cum_yaw

        lin = speed if linear_mode else 0.0
        ang = 0.0 if linear_mode else speed
        t_start = time.time()
        while time.time() - t_start < args.duration:
            node.cmd(lin, ang)
            rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = time.time() - t_start
        node.stop()
        spin_for(node, 0.4)  # let odom settle

        if linear_mode:
            dist = math.hypot(node.x - x0, node.y - y0)
            actual = dist / elapsed
            measured = f"odom distance {dist:.3f} m in {elapsed:.2f}s"
        else:
            rot = node.cum_yaw - yaw0
            actual = abs(rot) / elapsed
            measured = (f"odom rotation {math.degrees(rot):.1f} deg "
                        f"({rot:.3f} rad) in {elapsed:.2f}s")

        print("\n========== velocity calibration ==========")
        print(f"  commanded : {abs(speed):.3f} {unit}")
        print(f"  measured  : {actual:.3f} {unit}   [{measured}]")
        if actual > 1e-6:
            ratio = abs(speed) / actual
            key = 'linear_velocity_scale' if linear_mode else 'angular_velocity_scale'
            print(f"  cmd/actual ratio : {ratio:.3f}")
            if args.current_scale is not None:
                print(f"\n  -> set  {key} = {args.current_scale * ratio:.1f}"
                      f"   (current {args.current_scale:g} x {ratio:.3f})")
            else:
                print(f"\n  -> new {key} = current_scale x {ratio:.3f}"
                      f"   (pass --current-scale to get the number)")
        else:
            print("  measured ~0 - did the robot move? check bridge / power / scale")
        print("  (odom-based; cross-check with a tape measure for ground truth)")
        print("==========================================\n")
        return 0
    finally:
        try:
            node.stop()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
