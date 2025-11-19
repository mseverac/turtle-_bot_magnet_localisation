#!/usr/bin/env python3
"""
ros2_pose_recorder.py (with odometry overlay + magnet grid)

Subscribes to:
 - /robot_pose (geometry_msgs/PoseWithCovarianceStamped)
 - /joint_states (sensor_msgs/JointState)

Records /robot_pose as before and additionally computes an odometry
trajectory from wheel joint positions (wheel_left_joint, wheel_right_joint).

New features:
 - draw grid points (grey circles) every magnet_spacing meters (default 0.055 m)
   with a marker radius of magnet_point_radius (default 0.01 m).
 - save odometry-only x_vs_time and y_vs_time plots.

Usage:
    python3 ros2_pose_recorder.py
"""

import os
import threading
import math
import datetime
from typing import List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import JointState

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.patches import Circle

# --- PARAMETERS FOR ODOMETRY (adjust if needed) ---
WHEEL_RADIUS = 0.0215   # meters (e.g. 21.5 mm)
TRACK_GAUGE = 0.112     # meters (distance between wheels)
LEFT_JOINT_NAME = "wheel_left_joint"
RIGHT_JOINT_NAME = "wheel_right_joint"

# Grid / magnet visualization parameters
MAGNET_SPACING = 0.055    # meters between grid points
MAGNET_POINT_RADIUS = 0.01  # meters radius for visual circle
# -------------------------------------------------


def _quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _convert_wheel_angle_if_needed(a: float) -> float:
    """Heuristic: if magnitude > 2pi assume degrees and convert to radians."""
    if abs(a) > 2.0 * math.pi:
        return a * math.pi / 180.0
    return a


class PoseRecorder(Node):
    def __init__(self, topic_pose: str = '/robot_pose', topic_joint: str = '/joint_states'):
        super().__init__('pose_recorder')

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, topic_pose, self._pose_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, topic_joint, self._joint_callback, 50)

        # Recorded pose data (from EKF)
        self._lock = threading.Lock()
        self.times: List[float] = []
        self.xs: List[float] = []
        self.ys: List[float] = []
        self.thetas: List[float] = []
        self.covs: List[np.ndarray] = []  # each (6,6)

        # Odometry data (computed locally from /joint_states)
        self.times_odom: List[float] = []
        self.xs_odom: List[float] = []
        self.ys_odom: List[float] = []
        self.thetas_odom: List[float] = []

        # odometry internal state
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._prev_left_pos: Optional[float] = None
        self._prev_right_pos: Optional[float] = None
        self._prev_time_odom: Optional[float] = None
        self._odom_initialized = False

        # geometry
        self.r = WHEEL_RADIUS
        self.L = TRACK_GAUGE

        self.get_logger().info(f'Subscribed to: {topic_pose} and {topic_joint}')
        self.get_logger().info(f'Odom params: wheel_radius={self.r} m, track_gauge={self.L} m')
        self.get_logger().info(f'Grid params: spacing={MAGNET_SPACING} m, point radius={MAGNET_POINT_RADIUS} m')

    # ------------------------ Pose callback ------------------------
    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        stamp = msg.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = _quat_to_yaw(msg.pose.pose.orientation)

        cov_list = list(msg.pose.covariance)
        cov = np.array(cov_list).reshape((6, 6))

        with self._lock:
            self.times.append(t)
            self.xs.append(px)
            self.ys.append(py)
            self.thetas.append(yaw)
            self.covs.append(cov)

    # ------------------------ JointState callback & odom ------------------------
    def _joint_callback(self, msg: JointState):
        """Compute odometry from joint_state absolute wheel positions."""
        # find left/right indices
        try:
            left_it = msg.name.index(LEFT_JOINT_NAME)
            right_it = msg.name.index(RIGHT_JOINT_NAME)
            left_pos_raw = msg.position[left_it]
            right_pos_raw = msg.position[right_it]
        except ValueError:
            # fallback: if names not found, try first two positions
            if len(msg.position) >= 2:
                left_pos_raw = msg.position[0]
                right_pos_raw = msg.position[1]
            else:
                self.get_logger().debug('JointState: wheel names not found and not enough positions.')
                return

        left_pos = _convert_wheel_angle_if_needed(left_pos_raw)
        right_pos = _convert_wheel_angle_if_needed(right_pos_raw)

        # timestamp
        stamp = msg.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        # initialize previous positions on first message
        if not self._odom_initialized:
            self._prev_left_pos = left_pos
            self._prev_right_pos = right_pos
            self._prev_time_odom = t
            self._odom_initialized = True

            with self._lock:
                self.times_odom.append(t)
                self.xs_odom.append(self._odom_x)
                self.ys_odom.append(self._odom_y)
                self.thetas_odom.append(self._odom_theta)
            return

        # compute deltas
        delta_left = left_pos - self._prev_left_pos
        delta_right = right_pos - self._prev_right_pos

        self._prev_left_pos = left_pos
        self._prev_right_pos = right_pos

        # convert to linear displacements (arc lengths)
        dist_left = delta_left * self.r
        dist_right = delta_right * self.r

        delta_d = 0.5 * (dist_left + dist_right)
        delta_theta = (dist_right - dist_left) / self.L

        # integrate odometry (simple Euler as in EKF)
        theta = self._odom_theta
        self._odom_x += delta_d * math.cos(theta)
        self._odom_y += delta_d * math.sin(theta)
        self._odom_theta += delta_theta
        self._odom_theta = math.atan2(math.sin(self._odom_theta), math.cos(self._odom_theta))

        # store odom sample
        with self._lock:
            self.times_odom.append(t)
            self.xs_odom.append(self._odom_x)
            self.ys_odom.append(self._odom_y)
            self.thetas_odom.append(self._odom_theta)

    # ------------------------ Utility / Saving ------------------------
    def has_data(self) -> bool:
        with self._lock:
            return len(self.times) > 0 or len(self.times_odom) > 0

    def save_all(self, outdir: str = None):
        with self._lock:
            if len(self.times) == 0 and len(self.times_odom) == 0:
                self.get_logger().warning('No data recorded — nothing to save.')
                return

            times = np.array(self.times) if len(self.times) > 0 else np.array([])
            xs = np.array(self.xs) if len(self.xs) > 0 else np.array([])
            ys = np.array(self.ys) if len(self.ys) > 0 else np.array([])
            thetas = np.array(self.thetas) if len(self.thetas) > 0 else np.array([])
            covs = np.array(self.covs) if len(self.covs) > 0 else np.zeros((0,6,6))

            times_odom = np.array(self.times_odom) if len(self.times_odom) > 0 else np.array([])
            xs_odom = np.array(self.xs_odom) if len(self.xs_odom) > 0 else np.array([])
            ys_odom = np.array(self.ys_odom) if len(self.ys_odom) > 0 else np.array([])
            thetas_odom = np.array(self.thetas_odom) if len(self.thetas_odom) > 0 else np.array([])

        # create output directory
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        if outdir is None:
            outdir = os.path.join(os.getcwd(), f'pose_recording_{ts}')
        os.makedirs(outdir, exist_ok=True)

        # Save CSV (merge pose and odom rows by time index)
        rows = []
        N = max(len(times), len(times_odom))
        for i in range(N):
            row = {}
            if i < len(times):
                row['time_pose'] = float(times[i])
                row['x_pose'] = float(xs[i])
                row['y_pose'] = float(ys[i])
                row['theta_pose'] = float(thetas[i])
                cov_i = covs[i] if covs.shape[0] > i else np.zeros((6,6))
                row['cov_xx'] = float(cov_i[0, 0])
                row['cov_xy'] = float(cov_i[0, 1])
                row['cov_yy'] = float(cov_i[1, 1])
                row['cov_thetatheta'] = float(cov_i[5,5]) if cov_i.shape == (6,6) else float('nan')
            else:
                row.update({'time_pose': float('nan'), 'x_pose': float('nan'), 'y_pose': float('nan'), 'theta_pose': float('nan'),
                            'cov_xx': float('nan'), 'cov_xy': float('nan'), 'cov_yy': float('nan'), 'cov_thetatheta': float('nan')})

            if i < len(times_odom):
                row['time_odom'] = float(times_odom[i])
                row['x_odom'] = float(xs_odom[i])
                row['y_odom'] = float(ys_odom[i])
                row['theta_odom'] = float(thetas_odom[i])
            else:
                row.update({'time_odom': float('nan'), 'x_odom': float('nan'), 'y_odom': float('nan'), 'theta_odom': float('nan')})
            rows.append(row)

        df = pd.DataFrame(rows)
        csv_path = os.path.join(outdir, 'trajectory_combined.csv')
        df.to_csv(csv_path, index=False)
        self.get_logger().info(f'Saved CSV -> {csv_path}')

        # Also save odom-only CSV for convenience
        if len(times_odom) > 0:
            df_odom = pd.DataFrame({
                'time': times_odom,
                'x_odom': xs_odom,
                'y_odom': ys_odom,
                'theta_odom': thetas_odom
            })
            odom_csv = os.path.join(outdir, 'trajectory_odom.csv')
            df_odom.to_csv(odom_csv, index=False)
            self.get_logger().info(f'Saved odometry CSV -> {odom_csv}')

        # --- plotting ---
        # compute extents for grid
        all_x = np.concatenate([xs if xs.size else np.array([]), xs_odom if xs_odom.size else np.array([])])
        all_y = np.concatenate([ys if ys.size else np.array([]), ys_odom if ys_odom.size else np.array([])])
        if all_x.size > 0 and all_y.size > 0:
            xmin, xmax = np.min(all_x), np.max(all_x)
            ymin, ymax = np.min(all_y), np.max(all_y)
        else:
            xmin, xmax, ymin, ymax = -1.0, 1.0, -1.0, 1.0

        margin = 0.02  # meters margin around data for plotting grid
        gx_min = xmin - margin
        gx_max = xmax + margin
        gy_min = ymin - margin
        gy_max = ymax + margin

        # prepare grid points (avoid creating insanely many circles)
        # prepare grid points aligned to multiples of MAGNET_SPACING and including origin (0,0)
        def _aligned_bounds(minv, maxv, spacing):
            start = math.floor(minv / spacing) * spacing
            end = math.ceil(maxv / spacing) * spacing
            # ensure 0 is included
            start = min(start, 0.0)
            end = max(end, 0.0)
            return start, end

        gx_start, gx_end = _aligned_bounds(gx_min, gx_max, MAGNET_SPACING)
        gy_start, gy_end = _aligned_bounds(gy_min, gy_max, MAGNET_SPACING)

        xs_grid = np.arange(gx_start, gx_end + 1e-9, MAGNET_SPACING)
        ys_grid = np.arange(gy_start, gy_end + 1e-9, MAGNET_SPACING)
        grid_points = [(xx, yy) for xx in xs_grid for yy in ys_grid]


        # 1) XY trajectory with covariance ellipses, overlay odom in red and grid points in grey
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111)
        # plot grid circles first (light grey)
        for (gx, gy) in grid_points:
            circ = Circle((gx, gy), MAGNET_POINT_RADIUS, facecolor='lightgray', edgecolor='gray', alpha=0.6, linewidth=0.5)
            ax.add_patch(circ)

        if len(xs) > 0:
            ax.plot(xs, ys, '-o', markersize=3, linewidth=1, label='pose (EKF)', color='tab:blue')
        if len(xs_odom) > 0:
            ax.plot(xs_odom, ys_odom, '-o', markersize=3, linewidth=1, color='red', label='odom (local)')

        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title('Trajectory (XY): EKF (blue) vs Odom (red)')
        ax.axis('equal')

        # draw covariance ellipses at a subset of points (to avoid clutter)
        if len(xs) > 0:
            Np = len(xs)
            max_ellipses = 30
            step = max(1, Np // max_ellipses)
            for i in range(0, Np, step):
                cov2 = covs[i][0:2, 0:2] if covs.shape[0] > i else np.zeros((2,2))
                self._draw_cov_ellipse(ax, xs[i], ys[i], cov2, n_std=2.4477, edgecolor='tab:green')

        ax.legend()
        traj_png = os.path.join(outdir, 'trajectory.png')
        fig.savefig(traj_png, bbox_inches='tight', dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Saved trajectory plot -> {traj_png}')

        # 2) x vs time (pose) and overlay odom x (if available)
        if len(times) > 0:
            t0 = times[0]
        elif len(times_odom) > 0:
            t0 = times_odom[0]
        else:
            t0 = 0.0

        fig, ax = plt.subplots()
        if len(times) > 0:
            ax.plot(times - t0, xs, '-o', markersize=3, label='x (pose)')
        if len(times_odom) > 0:
            ax.plot(times_odom - t0, xs_odom, '-o', markersize=3, color='red', label='x (odom)')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('x (m)')
        ax.set_title('x vs time (pose and odom)')
        ax.legend()
        xpng = os.path.join(outdir, 'x_vs_time.png')
        fig.savefig(xpng, bbox_inches='tight', dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Saved x vs time -> {xpng}')

        # NEW: 2b) odom-only x vs time (separate file)
        if len(times_odom) > 0:
            fig, ax = plt.subplots()
            ax.plot(times_odom - t0, xs_odom, '-o', markersize=3, color='red', label='x (odom)')
            ax.set_xlabel('time (s)')
            ax.set_ylabel('x_odom (m)')
            ax.set_title('Odometry x vs time')
            ax.legend()
            xodom_png = os.path.join(outdir, 'x_vs_time_odom.png')
            fig.savefig(xodom_png, bbox_inches='tight', dpi=150)
            plt.close(fig)
            self.get_logger().info(f'Saved odom x vs time -> {xodom_png}')

        # 3) y vs time (pose) and overlay odom y
        fig, ax = plt.subplots()
        if len(times) > 0:
            ax.plot(times - t0, ys, '-o', markersize=3, label='y (pose)')
        if len(times_odom) > 0:
            ax.plot(times_odom - t0, ys_odom, '-o', markersize=3, color='red', label='y (odom)')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('y (m)')
        ax.set_title('y vs time (pose and odom)')
        ax.legend()
        ypng = os.path.join(outdir, 'y_vs_time.png')
        fig.savefig(ypng, bbox_inches='tight', dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Saved y vs time -> {ypng}')

        # NEW: 3b) odom-only y vs time (separate file)
        if len(times_odom) > 0:
            fig, ax = plt.subplots()
            ax.plot(times_odom - t0, ys_odom, '-o', markersize=3, color='red', label='y (odom)')
            ax.set_xlabel('time (s)')
            ax.set_ylabel('y_odom (m)')
            ax.set_title('Odometry y vs time')
            ax.legend()
            yodom_png = os.path.join(outdir, 'y_vs_time_odom.png')
            fig.savefig(yodom_png, bbox_inches='tight', dpi=150)
            plt.close(fig)
            self.get_logger().info(f'Saved odom y vs time -> {yodom_png}')

        # 4) theta vs time
        fig, ax = plt.subplots()
        if len(times) > 0:
            ax.plot(times - t0, thetas, '-o', markersize=3, label='theta (pose)')
        if len(times_odom) > 0:
            ax.plot(times_odom - t0, thetas_odom, '-o', markersize=3, color='red', label='theta (odom)')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('theta (rad)')
        ax.set_title('theta vs time')
        ax.legend()
        thetapng = os.path.join(outdir, 'theta_vs_time.png')
        fig.savefig(thetapng, bbox_inches='tight', dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Saved theta vs time -> {thetapng}')

        # 5) covariances vs time (sigma_x, sigma_y, sigma_theta) (pose only)
        if covs.shape[0] > 0:
            sig_x = np.sqrt(np.maximum(covs[:, 0, 0], 0.0))
            sig_y = np.sqrt(np.maximum(covs[:, 1, 1], 0.0))
            if covs.shape[1:] == (6, 6):
                sig_theta = np.sqrt(np.maximum(covs[:, 5, 5], 0.0))
            else:
                sig_theta = np.zeros_like(sig_x)

            fig, ax = plt.subplots()
            ax.plot(times - t0, sig_x, '-o', markersize=3, label='sigma_x')
            ax.plot(times - t0, sig_y, '-o', markersize=3, label='sigma_y')
            ax.set_xlabel('time (s)')
            ax.set_ylabel('std dev (m)')
            ax.set_title('Position uncertainty (std dev)')
            ax.legend()
            covxy_png = os.path.join(outdir, 'cov_x_y_vs_time.png')
            fig.savefig(covxy_png, bbox_inches='tight', dpi=150)
            plt.close(fig)
            self.get_logger().info(f'Saved cov x/y vs time -> {covxy_png}')

            fig, ax = plt.subplots()
            ax.plot(times - t0, sig_theta, '-o', markersize=3)
            ax.set_xlabel('time (s)')
            ax.set_ylabel('std dev (rad)')
            ax.set_title('Theta uncertainty (std dev)')
            covth_png = os.path.join(outdir, 'cov_theta_vs_time.png')
            fig.savefig(covth_png, bbox_inches='tight', dpi=150)
            plt.close(fig)
            self.get_logger().info(f'Saved cov theta vs time -> {covth_png}')

        self.get_logger().info(f'All figures and CSV saved to: {outdir}')

    def _draw_cov_ellipse(self, ax, x, y, cov2: np.ndarray, n_std: float = 2.4477, edgecolor='tab:blue'):
        if cov2.shape != (2, 2):
            return
        vals, vecs = np.linalg.eigh(cov2)
        order = vals.argsort()[::-1]
        vals = vals[order]
        vecs = vecs[:, order]
        vals = np.maximum(vals, 0.0)
        theta = math.atan2(vecs[1, 0], vecs[0, 0])
        width = 2 * n_std * math.sqrt(vals[0]) if vals[0] > 0 else 0.0
        height = 2 * n_std * math.sqrt(vals[1]) if vals[1] > 0 else 0.0
        ell = Circle((x, y), 0)  # placeholder to avoid flake; will add via Ellipse-like patch
        from matplotlib.patches import Ellipse
        ell = Ellipse((x, y), width=width, height=height, angle=math.degrees(theta),
                      fill=False, linewidth=1.0, edgecolor=edgecolor, alpha=0.7)
        ax.add_patch(ell)


def main():
    rclpy.init()
    recorder = PoseRecorder(topic_pose='/robot_pose', topic_joint='/joint_states')

    # Spin in background thread so the main thread can wait for user input
    spin_thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
    spin_thread.start()

    print('\nPose recorder running. Publishers connected?')
    print('Press ENTER in this terminal to stop recording and save plots.')
    try:
        input()  # wait for ENTER
    except KeyboardInterrupt:
        pass

    print('Stopping recorder, shutting down ROS...')
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)

    if recorder.has_data():
        recorder.save_all()
    else:
        print('No data recorded.')

    try:
        recorder.destroy_node()
    except Exception:
        pass


if __name__ == '__main__':
    main()
