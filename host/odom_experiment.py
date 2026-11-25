#!/usr/bin/env python3
"""Standalone ROS 2 Humble experiment runner for Gazebo vs ESP32 odometry.

This node runs with ``use_sim_time=True`` and acts as the sole ``/cmd_vel``
publisher during an experiment:
  1. Wait for non-zero ``/clock`` and a stationary Gazebo ``/odom`` sample.
  2. Latch that sample's ``header`` + ``pose.pose`` into a ``PoseStamped`` and
     publish it on ``/esp32/odom_init`` (RELIABLE) every 0.5 s until an
     ``/esp32/odom`` (RELIABLE) message arrives with matching ``header.stamp``.
  3. Drive the selected scenario by publishing ``/cmd_vel`` at 10 Hz in
     simulation time for ``duration`` simulation seconds.
  4. Publish zero ``Twist()`` at 10 Hz in simulation time for ~0.8 s (at least
     8 messages) on completion or SIGINT so both Gazebo and ESP32 stop cleanly.
"""

import argparse
import copy
from enum import Enum, auto
import math
import signal
import sys
import time
from typing import Optional

from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


SCENARIOS = {
    "straight": {"linear_x": 0.15, "angular_z": 0.0, "duration": 8.0},
    "turn": {"linear_x": 0.0, "angular_z": 0.5, "duration": 7.0},
    "arc": {"linear_x": 0.15, "angular_z": 0.3, "duration": 10.0},
    "figure8": {
        "linear_x": 0.16,
        "angular_z": 0.4,
        "duration": 4.0 * math.pi / 0.4,
    },
}

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

STATIONARY_VEL_TOL = 1e-3


class Phase(Enum):
    WAIT_ODOM = auto()
    WAIT_INIT_ACK = auto()
    DRIVING = auto()
    STOPPING = auto()
    DONE = auto()


def _quat_to_yaw(pose: Pose) -> float:
    q = pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _format_pose(pose: Pose) -> str:
    yaw = _quat_to_yaw(pose)
    return (
        f"x={pose.position.x:+.4f} m, y={pose.position.y:+.4f} m, "
        f"yaw={yaw:+.4f} rad ({math.degrees(yaw):+.2f} deg)"
    )


def _stamps_equal(a, b) -> bool:
    return int(a.sec) == int(b.sec) and int(a.nanosec) == int(b.nanosec)


class OdomExperimentNode(Node):
    """Coordinates ESP32 odom init handshake and sim-time /cmd_vel execution."""

    def __init__(
        self,
        scenario: str,
        linear_x: float,
        angular_z: float,
        duration: float,
        init_timeout: float,
        cmd_rate_hz: float,
        init_retry_period: float,
        stop_duration: float,
    ) -> None:
        super().__init__(
            "odom_experiment",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True)
            ],
        )

        self.scenario = scenario
        self.linear_x = linear_x
        self.angular_z = angular_z
        self.duration = duration
        self.init_timeout = init_timeout
        self.cmd_rate_hz = cmd_rate_hz
        self.init_retry_period = init_retry_period
        self.stop_duration = stop_duration

        self.phase = Phase.WAIT_ODOM
        self.exit_code = 0
        self.interrupted = False
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._wait_start_mono = time.monotonic()

        self.init_pose_msg: Optional[PoseStamped] = None
        self.init_pub_count = 0
        self.latest_gz_odom: Optional[Odometry] = None
        self.latest_esp_odom: Optional[Odometry] = None

        self._drive_start_sim_ns: Optional[int] = None
        self._last_progress_sim_s = -1.0
        self._stop_start_sim_ns: Optional[int] = None
        self._stop_start_mono: Optional[float] = None
        self._stop_msg_count = 0
        self._cmd_timer = None
        self._init_retry_timer = None

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._on_gz_odom, RELIABLE_QOS
        )
        self.esp_odom_sub = self.create_subscription(
            Odometry, "/esp32/odom", self._on_esp_odom, RELIABLE_QOS
        )
        self.init_pub = self.create_publisher(
            PoseStamped, "/esp32/odom_init", RELIABLE_QOS
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", RELIABLE_QOS)

        # Steady-clock housekeeping handles init retries and init-timeout checks
        # even if /clock or /esp32/odom stops responding.
        self._housekeeping_timer = self.create_timer(
            0.05, self._on_housekeeping, clock=self._steady_clock
        )

        self.get_logger().info(
            f"Scenario '{self.scenario}': linear.x={self.linear_x:.3f} m/s, "
            f"angular.z={self.angular_z:.3f} rad/s, duration={self.duration:.2f} s "
            f"(sim time @ {self.cmd_rate_hz:.1f} Hz)"
        )
        self.get_logger().info(
            "Waiting for non-zero /clock and stationary /odom sample..."
        )

    @property
    def finished(self) -> bool:
        return self.phase == Phase.DONE

    def request_stop(self, reason: str) -> None:
        """Transition cleanly to stopping or immediate abort on repeated SIGINT."""
        if self.phase == Phase.DONE:
            return
        if self.phase in (Phase.WAIT_ODOM, Phase.WAIT_INIT_ACK):
            self.get_logger().warn(
                f"Interrupted ({reason}) before driving started; exiting."
            )
            self.exit_code = 130
            self._finish()
            return
        if self.phase == Phase.DRIVING:
            self.interrupted = True
            self.exit_code = 130
            self.get_logger().warn(
                f"Interrupted ({reason}) during run; ramping to zero /cmd_vel..."
            )
            self._enter_stopping()
            return
        if self.phase == Phase.STOPPING:
            self.get_logger().warn(
                f"Second interrupt ({reason}) during stop phase; forcing exit."
            )
            self.exit_code = 130
            self._finish()

    def _sim_now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _on_gz_odom(self, msg: Odometry) -> None:
        self.latest_gz_odom = msg
        if self.phase != Phase.WAIT_ODOM:
            return

        sim_ns = self._sim_now_ns()
        msg_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        if sim_ns <= 0 or msg_stamp_ns <= 0:
            return

        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        if abs(vx) >= STATIONARY_VEL_TOL or abs(wz) >= STATIONARY_VEL_TOL:
            return

        pose_stamped = PoseStamped()
        pose_stamped.header = copy.deepcopy(msg.header)
        pose_stamped.pose = copy.deepcopy(msg.pose.pose)
        self.init_pose_msg = pose_stamped

        self.phase = Phase.WAIT_INIT_ACK
        self._wait_start_mono = time.monotonic()

        self.get_logger().info(
            "Captured stationary /odom sample -> publishing /esp32/odom_init: "
            f"stamp={pose_stamped.header.stamp.sec}.{pose_stamped.header.stamp.nanosec:09d}, "
            f"frame='{pose_stamped.header.frame_id}', {_format_pose(pose_stamped.pose)}"
        )
        self._publish_init_pose()
        self._init_retry_timer = self.create_timer(
            self.init_retry_period,
            self._publish_init_pose,
            clock=self._steady_clock,
        )

    def _publish_init_pose(self) -> None:
        if self.phase != Phase.WAIT_INIT_ACK or self.init_pose_msg is None:
            return
        self.init_pub.publish(self.init_pose_msg)
        self.init_pub_count += 1
        if self.init_pub_count == 1 or self.init_pub_count % 4 == 0:
            stamp = self.init_pose_msg.header.stamp
            self.get_logger().info(
                f"Sent /esp32/odom_init (#{self.init_pub_count}, "
                f"stamp={stamp.sec}.{stamp.nanosec:09d}); waiting for matching /esp32/odom..."
            )

    def _on_esp_odom(self, msg: Odometry) -> None:
        self.latest_esp_odom = msg
        if self.phase != Phase.WAIT_INIT_ACK or self.init_pose_msg is None:
            return

        if not _stamps_equal(msg.header.stamp, self.init_pose_msg.header.stamp):
            return

        if self._init_retry_timer is not None:
            self._init_retry_timer.cancel()
            self._init_retry_timer = None

        self.get_logger().info(
            "Received matching /esp32/odom confirmation "
            f"(stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, "
            f"{_format_pose(msg.pose.pose)})."
        )
        self._start_driving()

    def _start_driving(self) -> None:
        sim_ns = self._sim_now_ns()
        if sim_ns <= 0:
            self.get_logger().error("Simulation clock became zero before driving.")
            self.exit_code = 1
            self._finish()
            return

        self.phase = Phase.DRIVING
        self._drive_start_sim_ns = sim_ns
        self._last_progress_sim_s = 0.0
        self.get_logger().info(
            f"Starting /cmd_vel execution at {self.cmd_rate_hz:.1f} Hz sim-time "
            f"for {self.duration:.2f} s..."
        )
        # Uses node ROS clock (use_sim_time=True), so timer period is sim time.
        self._cmd_timer = self.create_timer(
            1.0 / self.cmd_rate_hz, self._on_cmd_timer
        )

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def _command_for_elapsed(self, elapsed_s: float) -> tuple[float, float]:
        if self.scenario == "figure8":
            wz = abs(self.angular_z)
            return (self.linear_x, wz if elapsed_s < 0.5 * self.duration else -wz)
        return (self.linear_x, self.angular_z)

    def _on_cmd_timer(self) -> None:
        sim_ns = self._sim_now_ns()
        if self.phase == Phase.DRIVING:
            assert self._drive_start_sim_ns is not None
            elapsed_s = max(0.0, (sim_ns - self._drive_start_sim_ns) * 1e-9)
            if elapsed_s >= self.duration:
                self.get_logger().info(
                    f"Completed {self.duration:.2f} s sim-time drive; sending zero Twist..."
                )
                self._enter_stopping()
                return

            vx, wz = self._command_for_elapsed(elapsed_s)
            self._publish_cmd(vx, wz)
            if elapsed_s - self._last_progress_sim_s >= 1.0:
                self._last_progress_sim_s = math.floor(elapsed_s)
                gz_info = (
                    _format_pose(self.latest_gz_odom.pose.pose)
                    if self.latest_gz_odom is not None
                    else "n/a"
                )
                esp_info = (
                    _format_pose(self.latest_esp_odom.pose.pose)
                    if self.latest_esp_odom is not None
                    else "n/a"
                )
                self.get_logger().info(
                    f"[{elapsed_s:5.2f}/{self.duration:.2f} s sim] "
                    f"Gazebo({gz_info}) | ESP32({esp_info})"
                )
        elif self.phase == Phase.STOPPING:
            self._publish_cmd(0.0, 0.0)
            self._stop_msg_count += 1
            assert self._stop_start_sim_ns is not None
            stop_elapsed_s = max(
                0.0, (sim_ns - self._stop_start_sim_ns) * 1e-9
            )
            if (
                stop_elapsed_s >= self.stop_duration
                and self._stop_msg_count >= 5
            ):
                self._finish()

    def _enter_stopping(self) -> None:
        self.phase = Phase.STOPPING
        self._stop_start_sim_ns = self._sim_now_ns()
        self._stop_start_mono = time.monotonic()
        self._stop_msg_count = 0
        # Send immediate zero Twist before waiting for subsequent 10 Hz ticks.
        self._publish_cmd(0.0, 0.0)
        self._stop_msg_count += 1
        if self._cmd_timer is None:
            self._cmd_timer = self.create_timer(
                1.0 / self.cmd_rate_hz, self._on_cmd_timer
            )

    def _on_housekeeping(self) -> None:
        if self.phase in (Phase.WAIT_ODOM, Phase.WAIT_INIT_ACK):
            if self.init_timeout > 0.0:
                waited = time.monotonic() - self._wait_start_mono
                if waited >= self.init_timeout:
                    stage = (
                        "/clock and stationary /odom"
                        if self.phase == Phase.WAIT_ODOM
                        else "/esp32/odom init confirmation"
                    )
                    self.get_logger().error(
                        f"Timed out after {waited:.1f} s waiting for {stage}."
                    )
                    self.exit_code = 1
                    self._finish()
        elif self.phase == Phase.STOPPING and self._stop_start_mono is not None:
            # Guard against /clock stalling during shutdown.
            wall_cap = max(3.0, self.stop_duration * 4.0)
            if time.monotonic() - self._stop_start_mono >= wall_cap:
                self.get_logger().warn(
                    "Sim-time stop guard reached wall-clock limit; finishing."
                )
                self._finish()

    def _finish(self) -> None:
        if self.phase == Phase.DONE:
            return
        if self._init_retry_timer is not None:
            self._init_retry_timer.cancel()
            self._init_retry_timer = None
        if self._cmd_timer is not None:
            self._cmd_timer.cancel()
            self._cmd_timer = None
        if self._housekeeping_timer is not None:
            self._housekeeping_timer.cancel()
            self._housekeeping_timer = None
        self.phase = Phase.DONE
        self._log_summary()

    def _log_summary(self) -> None:
        self.get_logger().info("=== Experiment Summary ===")
        if self.init_pose_msg is not None:
            s = self.init_pose_msg.header.stamp
            self.get_logger().info(
                f"Init stamp={s.sec}.{s.nanosec:09d} | "
                f"pose: {_format_pose(self.init_pose_msg.pose)}"
            )
        if self.latest_gz_odom is not None:
            s = self.latest_gz_odom.header.stamp
            self.get_logger().info(
                f"Final Gazebo /odom (stamp={s.sec}.{s.nanosec:09d}): "
                f"{_format_pose(self.latest_gz_odom.pose.pose)}"
            )
        else:
            self.get_logger().info("Final Gazebo /odom: none received")

        if self.latest_esp_odom is not None:
            s = self.latest_esp_odom.header.stamp
            self.get_logger().info(
                f"Final /esp32/odom  (stamp={s.sec}.{s.nanosec:09d}): "
                f"{_format_pose(self.latest_esp_odom.pose.pose)}"
            )
        else:
            self.get_logger().info("Final /esp32/odom:  none received")

        if (
            self.latest_gz_odom is not None
            and self.latest_esp_odom is not None
        ):
            gz_p = self.latest_gz_odom.pose.pose
            esp_p = self.latest_esp_odom.pose.pose
            dx = esp_p.position.x - gz_p.position.x
            dy = esp_p.position.y - gz_p.position.y
            pos_err = math.hypot(dx, dy)
            dyaw = _wrap_angle(_quat_to_yaw(esp_p) - _quat_to_yaw(gz_p))
            self.get_logger().info(
                f"Delta (ESP32 - Gazebo): dx={dx:+.4f} m, dy={dy:+.4f} m, "
                f"dist={pos_err:.4f} m, dyaw={dyaw:+.4f} rad ({math.degrees(dyaw):+.2f} deg)"
            )


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a sim-time /cmd_vel experiment against Gazebo and ESP32 micro-ROS "
            "after synchronizing /esp32/odom_init with /esp32/odom."
        )
    )
    parser.add_argument(
        "--scenario",
        choices=sorted(SCENARIOS.keys()),
        default="straight",
        help="Motion scenario preset (default: straight).",
    )
    parser.add_argument(
        "--linear-x",
        type=float,
        default=None,
        help="Override linear.x velocity in m/s.",
    )
    parser.add_argument(
        "--angular-z",
        type=float,
        default=None,
        help="Override angular.z velocity in rad/s.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Override active driving duration in simulation seconds.",
    )
    parser.add_argument(
        "--init-timeout",
        type=float,
        default=20.0,
        help="Wall-clock timeout (s) waiting for /clock+/odom or /esp32/odom init ack (<=0 disables).",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="/cmd_vel publish rate in Hz simulation time (default: 10.0).",
    )
    parser.add_argument(
        "--init-retry-period",
        type=float,
        default=0.5,
        help="Re-publish period (s) for /esp32/odom_init before ack (default: 0.5).",
    )
    parser.add_argument(
        "--stop-duration",
        type=float,
        default=0.8,
        help="Sim-time duration (s) to publish zero Twist after run (default: 0.8).",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    preset = SCENARIOS[args.scenario]
    linear_x = (
        args.linear_x if args.linear_x is not None else preset["linear_x"]
    )
    angular_z = (
        args.angular_z if args.angular_z is not None else preset["angular_z"]
    )
    duration = (
        args.duration if args.duration is not None else preset["duration"]
    )

    if duration < 0.0:
        print("Error: --duration must be >= 0", file=sys.stderr)
        return 2
    if args.rate <= 0.0:
        print("Error: --rate must be > 0", file=sys.stderr)
        return 2
    if args.init_retry_period <= 0.0:
        print("Error: --init-retry-period must be > 0", file=sys.stderr)
        return 2
    if args.stop_duration <= 0.0:
        print("Error: --stop-duration must be > 0", file=sys.stderr)
        return 2

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = OdomExperimentNode(
        scenario=args.scenario,
        linear_x=linear_x,
        angular_z=angular_z,
        duration=duration,
        init_timeout=args.init_timeout,
        cmd_rate_hz=args.rate,
        init_retry_period=args.init_retry_period,
        stop_duration=args.stop_duration,
    )

    def _on_signal(signum, _frame):
        sig_name = signal.Signals(signum).name
        node.request_stop(sig_name)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
