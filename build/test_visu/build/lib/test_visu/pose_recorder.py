#!/usr/bin/env python3
"""
ros2_pose_recorder.py

Node that subscribes to /robot_pose (geometry_msgs/msg/PoseWithCovarianceStamped)
Records pose messages until the user presses ENTER in the terminal, then saves
plots and CSVs necessary to visualize the robot trajectory and uncertainty.

Usage:
    python3 ros2_pose_recorder.py

Dependencies:
    - rclpy (ROS2)
    - numpy
    - matplotlib
    - pandas (optional, for CSV)

What it saves (in a timestamped directory):
    - trajectory.png         (XY plot with covariance ellipses)
    - x_vs_time.png
    - y_vs_time.png
    - theta_vs_time.png
    - cov_x_y_vs_time.png    (sigma_x and sigma_y over time)
    - cov_theta_vs_time.png  (sigma_theta over time)
    - trajectory.csv         (time, x, y, theta, cov entries)

The node is intentionally simple and standalone so you can run it alongside
your EKF node that publishes /robot_pose.

"""

import os
import threading
import math
import datetime
from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np
import matplotlib
# Use Agg backend so this works on headless machines
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd


class PoseRecorder(Node):
    def __init__(self, topic: str = '/robot_pose'):
        super().__init__('pose_recorder')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            topic,
            self._callback,
            10)

        # Storage for recorded data
        self.times: List[float] = []          # seconds (epoch or ROS time)
        self.xs: List[float] = []
        self.ys: List[float] = []
        self.thetas: List[float] = []
        # store covariance as flattened 6x6 (36) or at least diagonal entries
        self.covs: List[np.ndarray] = []     # each is (6,6) numpy array

        self._lock = threading.Lock()
        self.get_logger().info(f'Subscribed to: {topic}')

    def _callback(self, msg: PoseWithCovarianceStamped):
        # Extract time in seconds (use header stamp if valid)
        stamp = msg.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        # extract yaw from quaternion
        q = msg.pose.pose.orientation
        # yaw extraction (assuming roll=pitch=0 or small)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # covariance is a list of 36 numbers (row-major 6x6)
        cov_list = list(msg.pose.covariance)
        cov = np.array(cov_list).reshape((6, 6))

        with self._lock:
            self.times.append(t)
            self.xs.append(px)
            self.ys.append(py)
            self.thetas.append(yaw)
            self.covs.append(cov)

    def has_data(self) -> bool:
        with self._lock:
            return len(self.times) > 0

    def save_all(self, outdir: str = None):
        """Generate plots and save CSV/PNGs to outdir. If outdir None, create a
        timestamped directory in the current working directory."""
        with self._lock:
            if len(self.times) == 0:
                self.get_logger().warning('No data recorded — nothing to save.')
                return

            times = np.array(self.times)
            xs = np.array(self.xs)
            ys = np.array(self.ys)
            thetas = np.array(self.thetas)
            covs = np.array(self.covs)  # shape (N,6,6)

        # create output directory
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        if outdir is None:
            outdir = os.path.join(os.getcwd(), f'pose_recording_{ts}')
        os.makedirs(outdir, exist_ok=True)

        # save CSV
        rows = []
        for i in range(len(times)):
            row = {
                'time': times[i],
                'x': float(xs[i]),
                'y': float(ys[i]),
                'theta': float(thetas[i])
            }
            # append covariance diag entries and some off-diagonals
            cov_i = covs[i]
            row.update({
                'cov_xx': float(cov_i[0, 0]),
                'cov_xy': float(cov_i[0, 1]),
                'cov_yy': float(cov_i[1, 1]),
                'cov_thetatheta': float(cov_i[5, 5]) if cov_i.shape == (6, 6) else float(np.nan)
            })
            rows.append(row)

        df = pd.DataFrame(rows)
        csv_path = os.path.join(outdir, 'trajectory.csv')
        df.to_csv(csv_path, index=False)
        self.get_logger().info(f'Saved CSV -> {csv_path}')

        # relative times for plotting
        t0 = times[0]
        rel_t = times - t0

        # 1) XY trajectory with covariance ellipses
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111)
        ax.plot(xs, ys, '-o', markersize=3, linewidth=1, label='trajectory')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title('Trajectory (XY)')
        ax.axis('equal')

        # draw covariance ellipses at a subset of points (to avoid clutter)
        N = len(xs)
        max_ellipses = 30
        step = max(1, N // max_ellipses)
        for i in range(0, N, step):
            cov2 = covs[i][0:2, 0:2]
            self._draw_cov_ellipse(ax, xs[i], ys[i], cov2, n_std=2.4477)

        ax.legend()
        traj_png = os.path.join(outdir, 'trajectory.png')
        fig.savefig(traj_png, bbox_inches='tight', dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Saved trajectory plot -> {traj_png}')

        # 2) x vs time
        fig, ax = plt.subplots()
        ax.plot(rel_t, xs, '-o', markersize=3)
        ax.set_xlabel('time (s)')
        ax.set_ylabel('x (m)')
        ax.set_title('x vs time')
        xpng = os.path.join(outdir, 'x_vs_time.png')
        fig.savefig(xpng, bbox_inches='tight', dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Saved x vs time -> {xpng}')

        # 3) y vs time
        fig, ax = plt.subplots()
        ax.plot(rel_t, ys, '-o', markersize=3)
        ax.set_xlabel('time (s)')
        ax.set_ylabel('y (m)')
        ax.set_title('y vs time')
        ypng = os.path.join(outdir, 'y_vs_time.png')
        fig.savefig(ypng, bbox_inches='tight', dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Saved y vs time -> {ypng}')

        # 4) theta vs time
        fig, ax = plt.subplots()
        ax.plot(rel_t, thetas, '-o', markersize=3)
        ax.set_xlabel('time (s)')
        ax.set_ylabel('theta (rad)')
        ax.set_title('theta vs time')
        thetapng = os.path.join(outdir, 'theta_vs_time.png')
        fig.savefig(thetapng, bbox_inches='tight', dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Saved theta vs time -> {thetapng}')

        # 5) covariances vs time (sigma_x, sigma_y, sigma_theta)
        sig_x = np.sqrt(np.maximum(covs[:, 0, 0], 0.0))
        sig_y = np.sqrt(np.maximum(covs[:, 1, 1], 0.0))
        # theta covariance is usually at index (5,5) in 6x6 PoseWithCov
        if covs.shape[1:] == (6, 6):
            sig_theta = np.sqrt(np.maximum(covs[:, 5, 5], 0.0))
        else:
            sig_theta = np.zeros_like(sig_x)

        fig, ax = plt.subplots()
        ax.plot(rel_t, sig_x, '-o', markersize=3, label='sigma_x')
        ax.plot(rel_t, sig_y, '-o', markersize=3, label='sigma_y')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('std dev (m)')
        ax.set_title('Position uncertainty (std dev)')
        ax.legend()
        covxy_png = os.path.join(outdir, 'cov_x_y_vs_time.png')
        fig.savefig(covxy_png, bbox_inches='tight', dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Saved cov x/y vs time -> {covxy_png}')

        fig, ax = plt.subplots()
        ax.plot(rel_t, sig_theta, '-o', markersize=3)
        ax.set_xlabel('time (s)')
        ax.set_ylabel('std dev (rad)')
        ax.set_title('Theta uncertainty (std dev)')
        covth_png = os.path.join(outdir, 'cov_theta_vs_time.png')
        fig.savefig(covth_png, bbox_inches='tight', dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Saved cov theta vs time -> {covth_png}')

        self.get_logger().info(f'All figures and CSV saved to: {outdir}')

    def _draw_cov_ellipse(self, ax, x, y, cov2: np.ndarray, n_std: float = 2.4477, edgecolor='tab:blue'):
        """Draw an ellipse corresponding to the 2x2 covariance matrix cov2
        centered at (x,y) on axes ax. n_std is the chi-square scaling (default ~95%)."""
        # Ensure 2x2
        if cov2.shape != (2, 2):
            return
        # Eigen decomposition
        vals, vecs = np.linalg.eigh(cov2)
        # sort by largest
        order = vals.argsort()[::-1]
        vals = vals[order]
        vecs = vecs[:, order]

        # protect against tiny/negative eigenvalues
        vals = np.maximum(vals, 0.0)
        theta = math.atan2(vecs[1, 0], vecs[0, 0])
        width = 2 * n_std * math.sqrt(vals[0])
        height = 2 * n_std * math.sqrt(vals[1])

        from matplotlib.patches import Ellipse
        ell = Ellipse((x, y), width=width, height=height, angle=math.degrees(theta), fill=False, linewidth=1.0, edgecolor=edgecolor, alpha=0.7)
        ax.add_patch(ell)


def main():
    rclpy.init()
    recorder = PoseRecorder(topic='/robot_pose')

    # Spin in background thread so the main thread can wait for user input
    spin_thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
    spin_thread.start()

    print('\nPose recorder running. Publishers connected?')
    print('Press ENTER in this terminal to stop recording and save plots.')
    try:
        input()  # wait for ENTER
    except KeyboardInterrupt:
        # allow Ctrl-C as an alternative
        pass

    print('Stopping recorder, shutting down ROS...')
    # trigger shutdown of ROS spinning
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)

    # After shutdown, save data
    if recorder.has_data():
        recorder.save_all()
    else:
        print('No data recorded.')

    # destroy node explicitly
    try:
        recorder.destroy_node()
    except Exception:
        pass


if __name__ == '__main__':
    main()
