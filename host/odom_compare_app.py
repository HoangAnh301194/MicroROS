#!/usr/bin/env python3
"""Real-time Gazebo (/odom) vs ESP32 (/esp32/odom) odometry comparison GUI.

Subscribes to `/odom` and `/esp32/odom` (RELIABLE QoS), aligns samples strictly
by `header.stamp`, linearly interpolates Gazebo reference poses to each ESP32
timestamp (with shortest-angle yaw interpolation), and plots XY trajectories,
position error, and yaw error in a Tkinter + Matplotlib desktop window.

Use `--self-test` to validate all math, matching, interpolation, wrapping,
reset, and waiting-status logic without a ROS 2 graph or X11 display.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import dataclass
import math
import sys
import threading
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class OdomSample:
    """Single 2D odometry sample extracted from `nav_msgs/msg/Odometry`."""

    t_ns: int
    x: float
    y: float
    yaw_rad: float


@dataclass(frozen=True)
class MatchedSample:
    """ESP32 sample paired with time-interpolated Gazebo reference pose."""

    t_ns: int
    t_rel_s: float
    x_gz: float
    y_gz: float
    yaw_gz_rad: float
    x_esp: float
    y_esp: float
    yaw_esp_rad: float
    pos_err_m: float
    yaw_err_deg: float


@dataclass(frozen=True)
class ComparisonResult:
    """Computed matched series, status, and summary metrics for display."""

    status: str  # "waiting" or "active"
    status_detail: str
    gz_count: int
    esp_count: int
    matched: List[MatchedSample]
    final_pos_err_m: Optional[float]
    final_yaw_err_deg: Optional[float]
    pos_rmse_m: Optional[float]


def stamp_to_ns(sec: int, nanosec: int) -> int:
    """Convert ROS 2 Time `(sec, nanosec)` into integer nanoseconds."""
    return int(sec) * 1_000_000_000 + int(nanosec)


def quat_to_yaw_rad(x: float, y: float, z: float, w: float) -> float:
    """Extract planar yaw (radians) from quaternion `(x, y, z, w)`."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_rad(angle_rad: float) -> float:
    """Wrap an angle in radians into `[-pi, pi]`."""
    wrapped = (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped == -math.pi and angle_rad > 0.0:
        return math.pi
    return wrapped


def wrap_deg(angle_deg: float) -> float:
    """Wrap an angle in degrees into `[-180.0, 180.0]`."""
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped == -180.0 and angle_deg > 0.0:
        return 180.0
    return wrapped


def interpolate_odom_sample(
    s0: OdomSample, s1: OdomSample, target_t_ns: int
) -> OdomSample:
    """Linearly interpolate `(x, y, yaw)` between bracketing samples `s0` and `s1`."""
    dt_ns = s1.t_ns - s0.t_ns
    if dt_ns <= 0 or target_t_ns <= s0.t_ns:
        return OdomSample(target_t_ns, s0.x, s0.y, wrap_rad(s0.yaw_rad))
    if target_t_ns >= s1.t_ns:
        return OdomSample(target_t_ns, s1.x, s1.y, wrap_rad(s1.yaw_rad))

    alpha = (target_t_ns - s0.t_ns) / float(dt_ns)
    x = s0.x + alpha * (s1.x - s0.x)
    y = s0.y + alpha * (s1.y - s0.y)
    # Shortest-angle interpolation prevents artificial 360-deg sweeps across +-pi.
    yaw_rad = wrap_rad(s0.yaw_rad + alpha * wrap_rad(s1.yaw_rad - s0.yaw_rad))
    return OdomSample(target_t_ns, x, y, yaw_rad)


def match_trajectories(
    gz_samples: Sequence[OdomSample], esp_samples: Sequence[OdomSample]
) -> List[MatchedSample]:
    """Interpolate sorted `gz_samples` onto each bracketed `esp_samples` stamp."""
    if not gz_samples or not esp_samples:
        return []

    gz_times = [s.t_ns for s in gz_samples]
    t_min = gz_times[0]
    t_max = gz_times[-1]

    raw_matches: List[Tuple[OdomSample, OdomSample]] = []
    for esp in esp_samples:
        t_esp = esp.t_ns
        if t_esp < t_min or t_esp > t_max:
            continue

        idx = bisect_right(gz_times, t_esp)
        if idx > 0 and gz_times[idx - 1] == t_esp:
            gz_interp = gz_samples[idx - 1]
        elif 0 < idx < len(gz_samples):
            gz_interp = interpolate_odom_sample(
                gz_samples[idx - 1], gz_samples[idx], t_esp
            )
        else:
            continue
        raw_matches.append((gz_interp, esp))

    if not raw_matches:
        return []

    origin_ns = raw_matches[0][1].t_ns
    matched: List[MatchedSample] = []
    for gz_interp, esp in raw_matches:
        pos_err = math.hypot(esp.x - gz_interp.x, esp.y - gz_interp.y)
        yaw_err_deg = wrap_deg(
            math.degrees(esp.yaw_rad) - math.degrees(gz_interp.yaw_rad)
        )
        matched.append(
            MatchedSample(
                t_ns=esp.t_ns,
                t_rel_s=(esp.t_ns - origin_ns) * 1e-9,
                x_gz=gz_interp.x,
                y_gz=gz_interp.y,
                yaw_gz_rad=gz_interp.yaw_rad,
                x_esp=esp.x,
                y_esp=esp.y,
                yaw_esp_rad=esp.yaw_rad,
                pos_err_m=pos_err,
                yaw_err_deg=yaw_err_deg,
            )
        )
    return matched


def evaluate_odometry(
    gz_samples: Sequence[OdomSample], esp_samples: Sequence[OdomSample]
) -> ComparisonResult:
    """Compute matched error series, summary metrics, and waiting/active status."""
    gz_count = len(gz_samples)
    esp_count = len(esp_samples)

    if gz_count == 0 and esp_count == 0:
        return ComparisonResult(
            status="waiting",
            status_detail="Waiting for /odom and /esp32/odom messages...",
            gz_count=0,
            esp_count=0,
            matched=[],
            final_pos_err_m=None,
            final_yaw_err_deg=None,
            pos_rmse_m=None,
        )
    if gz_count == 0:
        return ComparisonResult(
            status="waiting",
            status_detail=(
                f"Waiting for Gazebo /odom messages... (ESP32 samples: {esp_count})"
            ),
            gz_count=0,
            esp_count=esp_count,
            matched=[],
            final_pos_err_m=None,
            final_yaw_err_deg=None,
            pos_rmse_m=None,
        )
    if esp_count == 0:
        return ComparisonResult(
            status="waiting",
            status_detail=(
                f"Waiting for ESP32 /esp32/odom messages... "
                f"(Gazebo samples: {gz_count})"
            ),
            gz_count=gz_count,
            esp_count=0,
            matched=[],
            final_pos_err_m=None,
            final_yaw_err_deg=None,
            pos_rmse_m=None,
        )

    matched = match_trajectories(gz_samples, esp_samples)
    if not matched:
        return ComparisonResult(
            status="waiting",
            status_detail=(
                "Waiting for overlapping timestamps "
                f"(Gazebo: {gz_count}, ESP32: {esp_count})..."
            ),
            gz_count=gz_count,
            esp_count=esp_count,
            matched=[],
            final_pos_err_m=None,
            final_yaw_err_deg=None,
            pos_rmse_m=None,
        )

    sum_sq_err = sum(m.pos_err_m * m.pos_err_m for m in matched)
    pos_rmse_m = math.sqrt(sum_sq_err / len(matched))
    last = matched[-1]

    return ComparisonResult(
        status="active",
        status_detail=(
            f"Active | Matched: {len(matched)} "
            f"(Gazebo: {gz_count}, ESP32: {esp_count})"
        ),
        gz_count=gz_count,
        esp_count=esp_count,
        matched=matched,
        final_pos_err_m=last.pos_err_m,
        final_yaw_err_deg=last.yaw_err_deg,
        pos_rmse_m=pos_rmse_m,
    )


class OdomBufferStore:
    """Thread-safe sample buffer with backward-clock-jump reset handling."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._gz: List[OdomSample] = []
        self._esp: List[OdomSample] = []
        self._revision: int = 0

    def _append_sample(self, buf: List[OdomSample], sample: OdomSample) -> bool:
        """Append sample; return False if a backward timestamp jump requires reset."""
        if buf:
            last_t = buf[-1].t_ns
            if sample.t_ns < last_t:
                return False
            if sample.t_ns == last_t:
                buf[-1] = sample
                return True
        buf.append(sample)
        return True

    def add_gazebo(self, sample: OdomSample) -> bool:
        """Add a Gazebo `/odom` sample; returns True if a reset occurred."""
        with self._lock:
            reset = not self._append_sample(self._gz, sample)
            if reset:
                self._gz.clear()
                self._esp.clear()
                self._gz.append(sample)
            self._revision += 1
            return reset

    def add_esp32(self, sample: OdomSample) -> bool:
        """Add an ESP32 `/esp32/odom` sample; returns True if a reset occurred."""
        with self._lock:
            reset = not self._append_sample(self._esp, sample)
            if reset:
                self._gz.clear()
                self._esp.clear()
                self._esp.append(sample)
            self._revision += 1
            return reset

    def clear(self) -> None:
        """Clear both trajectory buffers for a manual reset."""
        with self._lock:
            self._gz.clear()
            self._esp.clear()
            self._revision += 1

    def snapshot(
        self, last_revision: Optional[int] = None
    ) -> Tuple[int, bool, List[OdomSample], List[OdomSample]]:
        """Return `(revision, changed, gz_copy, esp_copy)` under lock."""
        with self._lock:
            if last_revision is not None and self._revision == last_revision:
                return self._revision, False, [], []
            return self._revision, True, list(self._gz), list(self._esp)


def _sample_from_odom_msg(msg) -> OdomSample:
    """Extract `OdomSample` from a `nav_msgs.msg.Odometry` message."""
    stamp = msg.header.stamp
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    return OdomSample(
        t_ns=stamp_to_ns(stamp.sec, stamp.nanosec),
        x=float(pos.x),
        y=float(pos.y),
        yaw_rad=quat_to_yaw_rad(
            float(ori.x), float(ori.y), float(ori.z), float(ori.w)
        ),
    )


def run_gui(ros_args: Sequence[str]) -> int:
    """Initialize ROS 2 listener thread and Tkinter + Matplotlib TkAgg UI."""
    import signal
    import matplotlib

    matplotlib.use("TkAgg")
    from matplotlib.backends.backend_tkagg import (
        FigureCanvasTkAgg,
        NavigationToolbar2Tk,
    )
    from matplotlib.figure import Figure
    import tkinter as tk
    from tkinter import ttk

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    class OdomCompareNode(Node):
        def __init__(self, store: OdomBufferStore) -> None:
            super().__init__("odom_compare_app")
            self._store = store
            reliable_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
            )
            self.create_subscription(
                Odometry, "/odom", self._on_gazebo_odom, reliable_qos
            )
            self.create_subscription(
                Odometry, "/esp32/odom", self._on_esp32_odom, reliable_qos
            )

        def _on_gazebo_odom(self, msg: Odometry) -> None:
            if self._store.add_gazebo(_sample_from_odom_msg(msg)):
                self.get_logger().info(
                    "Detected backward timestamp jump on /odom; cleared buffers."
                )

        def _on_esp32_odom(self, msg: Odometry) -> None:
            if self._store.add_esp32(_sample_from_odom_msg(msg)):
                self.get_logger().info(
                    "Detected backward timestamp jump on /esp32/odom; cleared buffers."
                )

    class OdomCompareWindow:
        UPDATE_INTERVAL_MS = 250  # 4 Hz UI refresh to keep CPU light alongside Gazebo
        DEFAULT_FOLLOW_RADIUS_M = 0.8
        MIN_FOLLOW_RADIUS_M = 0.05
        MAX_FOLLOW_RADIUS_M = 50.0

        def __init__(
            self, root: tk.Tk, store: OdomBufferStore, sigint_event: threading.Event
        ) -> None:
            self.root = root
            self.store = store
            self._sigint_event = sigint_event
            self._last_revision: Optional[int] = None
            self._after_id: Optional[str] = None
            self._closing = False
            self._follow_radius_m = self.DEFAULT_FOLLOW_RADIUS_M

            self.root.title("Gazebo vs ESP32 Odometry Comparison")
            self.root.geometry("1120x820")

            self._build_controls()
            self._build_metrics_panel()
            self._build_plots()

            self._refresh_ui(force=True)
            self._schedule_next_update()

        def _build_controls(self) -> None:
            top = ttk.Frame(self.root, padding=(10, 8))
            top.pack(fill=tk.X)

            self.status_var = tk.StringVar(value="Starting...")
            status_lbl = ttk.Label(
                top,
                textvariable=self.status_var,
                font=("TkDefaultFont", 11, "bold"),
            )
            status_lbl.pack(side=tk.LEFT, fill=tk.X, expand=True)

            self.follow_var = tk.BooleanVar(value=True)
            self.zoom_var = tk.StringVar(value=f"±{self._follow_radius_m:.2f} m")

            reset_btn = ttk.Button(
                top, text="Clear / Reset", command=self._on_clear_reset
            )
            reset_btn.pack(side=tk.RIGHT, padx=(6, 0))

            zoom_out_btn = ttk.Button(
                top, text="Zoom -", width=7, command=lambda: self._adjust_zoom(1.25)
            )
            zoom_out_btn.pack(side=tk.RIGHT, padx=(4, 0))

            zoom_in_btn = ttk.Button(
                top, text="Zoom +", width=7, command=lambda: self._adjust_zoom(0.8)
            )
            zoom_in_btn.pack(side=tk.RIGHT, padx=(4, 0))

            zoom_lbl = ttk.Label(top, textvariable=self.zoom_var, width=9, anchor=tk.CENTER)
            zoom_lbl.pack(side=tk.RIGHT, padx=(4, 0))

            follow_chk = ttk.Checkbutton(
                top,
                text="Follow trajectory",
                variable=self.follow_var,
                command=self._on_follow_toggle,
            )
            follow_chk.pack(side=tk.RIGHT, padx=(8, 4))

        def _build_metrics_panel(self) -> None:
            panel = ttk.Frame(self.root, padding=(10, 4))
            panel.pack(fill=tk.X)

            self.final_pos_var = tk.StringVar(value="-- m")
            self.final_yaw_var = tk.StringVar(value="-- deg")
            self.rmse_var = tk.StringVar(value="-- m")

            cards = [
                ("Final Position Error", self.final_pos_var),
                ("Final Yaw Error", self.final_yaw_var),
                ("Position RMSE", self.rmse_var),
            ]
            for idx, (label_text, var) in enumerate(cards):
                box = ttk.LabelFrame(panel, text=label_text, padding=(12, 6))
                box.grid(row=0, column=idx, sticky="ew", padx=6)
                panel.columnconfigure(idx, weight=1)
                ttk.Label(
                    box, textvariable=var, font=("TkDefaultFont", 14, "bold")
                ).pack(anchor=tk.CENTER)

        def _build_plots(self) -> None:
            self.fig = Figure(figsize=(10.5, 6.8), dpi=100)
            gs = self.fig.add_gridspec(2, 2, height_ratios=[1.25, 1.0], hspace=0.34, wspace=0.26)

            self.ax_xy = self.fig.add_subplot(gs[0, :])
            self.ax_pos_err = self.fig.add_subplot(gs[1, 0])
            self.ax_yaw_err = self.fig.add_subplot(gs[1, 1])

            # 1. XY trajectories with equal axis scale
            self.ax_xy.set_title("XY Trajectory: Gazebo (/odom) vs ESP32 (/esp32/odom)")
            self.ax_xy.set_xlabel("x (m)")
            self.ax_xy.set_ylabel("y (m)")
            self.ax_xy.set_aspect("equal", adjustable="datalim")
            self.ax_xy.grid(True, linestyle="--", alpha=0.4)
            (self.line_gz_xy,) = self.ax_xy.plot(
                [],
                [],
                color="#1f77b4",
                linewidth=1.0,
                alpha=0.85,
                zorder=2,
                label="Gazebo /odom",
            )
            (self.line_esp_xy,) = self.ax_xy.plot(
                [],
                [],
                color="#d62728",
                linewidth=2.0,
                linestyle="--",
                zorder=3,
                label="ESP32 /esp32/odom",
            )
            (self.pt_gz_xy,) = self.ax_xy.plot(
                [],
                [],
                color="#1f77b4",
                marker="o",
                markersize=5,
                linestyle="None",
                zorder=4,
            )
            (self.pt_esp_xy,) = self.ax_xy.plot(
                [],
                [],
                color="#d62728",
                marker="o",
                markersize=7,
                markerfacecolor="none",
                markeredgewidth=1.8,
                linestyle="None",
                zorder=5,
            )
            self.ax_xy.legend(loc="best")

            # 2. Position error vs time
            self.ax_pos_err.set_title("Position Error vs Time")
            self.ax_pos_err.set_xlabel("time (s)")
            self.ax_pos_err.set_ylabel("position error (m)")
            self.ax_pos_err.grid(True, linestyle="--", alpha=0.4)
            (self.line_pos_err,) = self.ax_pos_err.plot(
                [], [], color="#2ca02c", linewidth=1.8
            )

            # 3. Yaw error vs time
            self.ax_yaw_err.set_title("Yaw Error vs Time")
            self.ax_yaw_err.set_xlabel("time (s)")
            self.ax_yaw_err.set_ylabel("yaw error (deg)")
            self.ax_yaw_err.grid(True, linestyle="--", alpha=0.4)
            self.ax_yaw_err.axhline(0.0, color="gray", linewidth=0.9, linestyle=":")
            (self.line_yaw_err,) = self.ax_yaw_err.plot(
                [], [], color="#9467bd", linewidth=1.8
            )

            self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
            self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=8, pady=(6, 0))
            self.toolbar = NavigationToolbar2Tk(self.canvas, self.root)
            self.toolbar.update()
            self.canvas.mpl_connect("scroll_event", self._on_scroll)
            self.canvas.draw_idle()

        def _on_clear_reset(self) -> None:
            self.store.clear()
            self._refresh_ui(force=True)

        def _on_follow_toggle(self) -> None:
            self._refresh_ui(force=True)

        def _adjust_zoom(self, factor: float) -> None:
            self._follow_radius_m = min(
                self.MAX_FOLLOW_RADIUS_M,
                max(self.MIN_FOLLOW_RADIUS_M, self._follow_radius_m * factor),
            )
            self.zoom_var.set(f"±{self._follow_radius_m:.2f} m")
            if not self.follow_var.get():
                self.follow_var.set(True)
            self._refresh_ui(force=True)

        def _on_scroll(self, event) -> None:
            if event.inaxes is not self.ax_xy:
                return
            if event.button == "up":
                self._adjust_zoom(0.8)
            elif event.button == "down":
                self._adjust_zoom(1.25)

        def _update_xy_view(
            self, gz_samples: Sequence[OdomSample], esp_samples: Sequence[OdomSample]
        ) -> None:
            if self.follow_var.get():
                latest_candidates = []
                if gz_samples:
                    latest_candidates.append(gz_samples[-1])
                if esp_samples:
                    latest_candidates.append(esp_samples[-1])
                if latest_candidates:
                    cx = sum(s.x for s in latest_candidates) / len(latest_candidates)
                    cy = sum(s.y for s in latest_candidates) / len(latest_candidates)
                    max_offset = max(
                        max(abs(s.x - cx), abs(s.y - cy)) for s in latest_candidates
                    )
                    r = max(self._follow_radius_m, max_offset * 1.35)
                    self.ax_xy.set_xlim(cx - r, cx + r)
                    self.ax_xy.set_ylim(cy - r, cy + r)
                    return
            if self.toolbar.mode == "":
                self.ax_xy.relim()
                self.ax_xy.autoscale_view()

        def _schedule_next_update(self) -> None:
            if not self._closing:
                self._after_id = self.root.after(
                    self.UPDATE_INTERVAL_MS, self._on_timer
                )

        def _on_timer(self) -> None:
            if self._sigint_event.is_set():
                self.shutdown()
                self.root.quit()
                self.root.destroy()
                return
            self._refresh_ui(force=False)
            self._schedule_next_update()

        def _refresh_ui(self, force: bool = False) -> None:
            rev, changed, gz_samples, esp_samples = self.store.snapshot(
                None if force else self._last_revision
            )
            if not changed and not force:
                return
            self._last_revision = rev

            result = evaluate_odometry(gz_samples, esp_samples)
            self.status_var.set(result.status_detail)

            if result.status == "active":
                assert result.final_pos_err_m is not None
                assert result.final_yaw_err_deg is not None
                assert result.pos_rmse_m is not None
                self.final_pos_var.set(f"{result.final_pos_err_m:.4f} m")
                self.final_yaw_var.set(f"{result.final_yaw_err_deg:+.2f} deg")
                self.rmse_var.set(f"{result.pos_rmse_m:.4f} m")
            else:
                self.final_pos_var.set("-- m")
                self.final_yaw_var.set("-- deg")
                self.rmse_var.set("-- m")

            # Plot raw trajectories as soon as samples arrive, even while waiting
            # for overlapping interpolation pairs.
            self.line_gz_xy.set_data(
                [s.x for s in gz_samples], [s.y for s in gz_samples]
            )
            self.line_esp_xy.set_data(
                [s.x for s in esp_samples], [s.y for s in esp_samples]
            )
            self.pt_gz_xy.set_data(
                [gz_samples[-1].x] if gz_samples else [],
                [gz_samples[-1].y] if gz_samples else [],
            )
            self.pt_esp_xy.set_data(
                [esp_samples[-1].x] if esp_samples else [],
                [esp_samples[-1].y] if esp_samples else [],
            )
            self._update_xy_view(gz_samples, esp_samples)

            times = [m.t_rel_s for m in result.matched]
            pos_errs = [m.pos_err_m for m in result.matched]
            yaw_errs = [m.yaw_err_deg for m in result.matched]

            self.line_pos_err.set_data(times, pos_errs)
            self.ax_pos_err.relim()
            self.ax_pos_err.autoscale_view()

            self.line_yaw_err.set_data(times, yaw_errs)
            self.ax_yaw_err.relim()
            self.ax_yaw_err.autoscale_view()

            self.canvas.draw_idle()

        def shutdown(self) -> None:
            self._closing = True
            if self._after_id is not None:
                try:
                    self.root.after_cancel(self._after_id)
                except tk.TclError:
                    pass
                self._after_id = None

    store = OdomBufferStore()
    rclpy.init(args=list(ros_args))
    node = OdomCompareNode(store)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    stop_event = threading.Event()
    sigint_event = threading.Event()
    prev_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, lambda _s, _f: sigint_event.set())

    def spin_ros() -> None:
        try:
            while not stop_event.is_set() and rclpy.ok():
                executor.spin_once(timeout_sec=0.05)
        except Exception:
            pass

    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()

    root = tk.Tk()
    window = OdomCompareWindow(root, store, sigint_event)
    closed = False

    def on_close() -> None:
        nonlocal closed
        if closed:
            return
        closed = True
        window.shutdown()
        stop_event.set()
        try:
            root.quit()
            root.destroy()
        except tk.TclError:
            pass

    root.protocol("WM_DELETE_WINDOW", on_close)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_close()
    finally:
        signal.signal(signal.SIGINT, prev_sigint)
        stop_event.set()
        ros_thread.join(timeout=1.0)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def run_self_test() -> int:
    """Run deterministic self-tests without ROS 2 or X11 display."""
    # 1. Quaternion -> yaw and angle wrapping across +-180 deg boundary.
    yaw_179 = quat_to_yaw_rad(
        0.0, 0.0, math.sin(math.radians(179.0) / 2.0), math.cos(math.radians(179.0) / 2.0)
    )
    yaw_neg179 = quat_to_yaw_rad(
        0.0, 0.0, math.sin(math.radians(-179.0) / 2.0), math.cos(math.radians(-179.0) / 2.0)
    )
    assert math.isclose(math.degrees(yaw_179), 179.0, abs_tol=1e-9)
    assert math.isclose(math.degrees(yaw_neg179), -179.0, abs_tol=1e-9)

    err_pos_to_neg = wrap_deg(math.degrees(yaw_179) - math.degrees(yaw_neg179))
    err_neg_to_pos = wrap_deg(math.degrees(yaw_neg179) - math.degrees(yaw_179))
    assert math.isclose(err_pos_to_neg, -2.0, abs_tol=1e-9), err_pos_to_neg
    assert math.isclose(err_neg_to_pos, 2.0, abs_tol=1e-9), err_neg_to_pos
    assert math.isclose(abs(err_pos_to_neg), 2.0, abs_tol=1e-9)
    assert math.isclose(abs(err_neg_to_pos), 2.0, abs_tol=1e-9)

    # 2. Exact linear interpolation of x, y and shortest-angle yaw across +-pi.
    gz = [
        OdomSample(stamp_to_ns(10, 0), 0.0, 0.0, math.radians(170.0)),
        OdomSample(stamp_to_ns(12, 0), 2.0, 4.0, math.radians(-170.0)),
    ]
    # Midpoint at t=11.0s: x=1.0, y=2.0, shortest-angle midpoint between +170 and -170 is +-180 deg.
    esp_interp = [
        OdomSample(stamp_to_ns(11, 0), 1.0, 2.0, math.radians(179.0)),
    ]
    res_interp = evaluate_odometry(gz, esp_interp)
    assert res_interp.status == "active"
    assert len(res_interp.matched) == 1
    m0 = res_interp.matched[0]
    assert math.isclose(m0.x_gz, 1.0, abs_tol=1e-12)
    assert math.isclose(m0.y_gz, 2.0, abs_tol=1e-12)
    assert math.isclose(abs(math.degrees(m0.yaw_gz_rad)), 180.0, abs_tol=1e-9)
    assert math.isclose(m0.pos_err_m, 0.0, abs_tol=1e-12)
    assert math.isclose(abs(m0.yaw_err_deg), 1.0, abs_tol=1e-9)

    # Quarter point at t=10.5s: x=0.5, y=1.0, yaw=175 deg; ESP at 179 deg vs -179 deg test.
    gz_wrap = [
        OdomSample(stamp_to_ns(0, 0), 0.0, 0.0, math.radians(-179.0)),
        OdomSample(stamp_to_ns(1, 0), 1.0, 0.0, math.radians(-179.0)),
    ]
    esp_wrap = [
        OdomSample(stamp_to_ns(0, 500_000_000), 0.5, 0.0, math.radians(179.0)),
    ]
    res_wrap = evaluate_odometry(gz_wrap, esp_wrap)
    assert res_wrap.status == "active"
    assert math.isclose(abs(res_wrap.final_yaw_err_deg), 2.0, abs_tol=1e-9)

    # 3. Known position errors -> verify final error and RMSE match analytical values.
    gz_traj = [
        OdomSample(stamp_to_ns(0, 0), 0.0, 0.0, 0.0),
        OdomSample(stamp_to_ns(1, 0), 1.0, 0.0, 0.0),
        OdomSample(stamp_to_ns(2, 0), 2.0, 0.0, 0.0),
        OdomSample(stamp_to_ns(3, 0), 3.0, 0.0, 0.0),
    ]
    # Errors at t=0.5, 1.5, 2.5 are: (0.3, 0.0)=0.3, (0.0, 0.4)=0.4, (0.3, 0.4)=0.5.
    esp_traj = [
        OdomSample(stamp_to_ns(0, 500_000_000), 0.8, 0.0, 0.0),
        OdomSample(stamp_to_ns(1, 500_000_000), 1.5, 0.4, 0.0),
        OdomSample(stamp_to_ns(2, 500_000_000), 2.8, 0.4, math.radians(5.0)),
    ]
    res_err = evaluate_odometry(gz_traj, esp_traj)
    expected_rmse = math.sqrt((0.3**2 + 0.4**2 + 0.5**2) / 3.0)
    assert res_err.status == "active"
    assert len(res_err.matched) == 3
    assert math.isclose(res_err.final_pos_err_m, 0.5, abs_tol=1e-12)
    assert math.isclose(res_err.final_yaw_err_deg, 5.0, abs_tol=1e-9)
    assert math.isclose(res_err.pos_rmse_m, expected_rmse, abs_tol=1e-12)
    assert math.isclose(res_err.matched[0].t_rel_s, 0.0, abs_tol=1e-12)
    assert math.isclose(res_err.matched[2].t_rel_s, 2.0, abs_tol=1e-12)

    # 4. Waiting status for empty, single-stream, and non-bracketing timestamps.
    assert evaluate_odometry([], []).status == "waiting"
    assert evaluate_odometry(gz_traj, []).status == "waiting"
    assert evaluate_odometry([], esp_traj).status == "waiting"
    non_bracketing_esp = [
        OdomSample(stamp_to_ns(4, 0), 4.0, 0.0, 0.0),
    ]
    res_unmatched = evaluate_odometry(gz_traj, non_bracketing_esp)
    assert res_unmatched.status == "waiting"
    assert len(res_unmatched.matched) == 0
    assert res_unmatched.final_pos_err_m is None
    assert res_unmatched.pos_rmse_m is None

    # 5. Backward clock jump auto-reset and manual clear in OdomBufferStore.
    store = OdomBufferStore()
    assert not store.add_gazebo(OdomSample(stamp_to_ns(5, 0), 1.0, 2.0, 0.0))
    assert not store.add_esp32(OdomSample(stamp_to_ns(5, 0), 1.1, 2.0, 0.0))
    assert store.add_gazebo(OdomSample(stamp_to_ns(0, 100_000_000), 0.0, 0.0, 0.0))
    _, _, gz_after, esp_after = store.snapshot()
    assert len(gz_after) == 1 and len(esp_after) == 0
    store.clear()
    _, _, gz_cleared, esp_cleared = store.snapshot()
    assert len(gz_cleared) == 0 and len(esp_cleared) == 0

    print("All odom_compare_app self-tests passed.")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Real-time Gazebo vs ESP32 odometry comparison app."
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run built-in unit tests without ROS 2 or an X11 display.",
    )
    args, ros_args = parser.parse_known_args(argv)
    if args.self_test:
        return run_self_test()
    return run_gui(ros_args)


if __name__ == "__main__":
    sys.exit(main())
