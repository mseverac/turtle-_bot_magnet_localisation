# Creating and running a small script that reads the provided circles.txt content,
# computes differential-drive odometry from the first two columns (wheel angles in degrees),
# and plots the resulting trajectory and wheel angles over samples.
#
# Assumptions made (based on earlier conversation context):
# - The two first columns are absolute wheel rotation angles in degrees (cumulative).
# - Wheel radius = 0.033 m (33 mm) and track gauge = 0.283 m (283 mm).
# - The file is sampled uniformly; we use index-based time for plots.
#
# This cell will:
# - write the provided sample content to circles.txt,
# - read it, compute pose, and save two PNGs in the working directory,
# - display the plots inline.
import math
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# write the provided content to circles.txt

name = "twoloops.txt"

src = Path(name)
if not src.exists():
    raise FileNotFoundError(name +" not found in working directory")
content = src.read_text()


# Read the file using pandas for robustness (tab separated, ignore extra columns)
df = pd.read_csv(name, sep=r'\s+', header=None, engine='python')

# Ensure we have at least two columns
if df.shape[1] < 2:
    raise RuntimeError("File must contain at least two columns for left and right wheel angles.")

# Extract first two columns: wheel angles in degrees (assumed absolute/cumulative)
left_deg = df.iloc[:, 0].to_numpy(dtype=float)
right_deg = df.iloc[:, 1].to_numpy(dtype=float)

# Convert to radians
left_rad = np.deg2rad(left_deg)
right_rad = np.deg2rad(right_deg)

# Robot parameters (taken from earlier conversation context)
wheel_radius = 0.0215    # meters
track_gauge = 0.112     # meters (distance between wheels)

# Initialize pose arrays
N = len(left_rad)
xs = np.zeros(N)
ys = np.zeros(N)
thetas = np.zeros(N)

# We'll interpret the columns as absolute wheel angles. Compute deltas between successive samples.
prev_left = left_rad[0]
prev_right = right_rad[0]
theta = 0.0
x = 0.0
y = 0.0
xs[0] = x
ys[0] = y
thetas[0] = theta

for i in range(1, N):
    delta_left = left_rad[i] - prev_left
    delta_right = right_rad[i] - prev_right
    prev_left = left_rad[i]
    prev_right = right_rad[i]
    # convert wheel rotation delta to linear displacement (arc length)
    dist_left = wheel_radius * delta_left
    dist_right = wheel_radius * delta_right
    delta_d = 0.5 * (dist_left + dist_right)
    delta_theta = (dist_right - dist_left) / track_gauge
    # update pose (simple first-order integration using current theta)
    x = x + delta_d * math.cos(theta)
    y = y + delta_d * math.sin(theta)
    theta = theta + delta_theta
    xs[i] = x
    ys[i] = y
    thetas[i] = theta

# Save and plot trajectory
traj_fig = plt.figure(figsize=(6,6))
ax = traj_fig.add_subplot(1,1,1)
ax.plot(xs, ys, marker='o', linestyle='-')
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_title('Robot trajectory from wheel angles (degrees)')
ax.axis('equal')
traj_png = 'trajectory_'+name+'.png'
traj_fig.savefig(traj_png, bbox_inches='tight', dpi=150)

# Plot wheel angles
fig2 = plt.figure(figsize=(6,4))
ax2 = fig2.add_subplot(1,1,1)
ax2.plot(np.arange(N), left_deg, marker='o', linestyle='-', label='left (deg)')
ax2.plot(np.arange(N), right_deg, marker='o', linestyle='-', label='right (deg)')
ax2.set_xlabel('sample index')
ax2.set_ylabel('angle (deg)')
ax2.set_title('Wheel angles')
ax2.legend()
angles_png = 'wheel_angles_'+name+'.png'
fig2.savefig(angles_png, bbox_inches='tight', dpi=150)

# Plot heading (theta) vs time (sample index)
fig3 = plt.figure(figsize=(6,4))
ax3 = fig3.add_subplot(1,1,1)
ax3.plot(np.arange(N), thetas, marker='o', linestyle='-', label='theta (rad)')
ax3.set_xlabel('sample index')
ax3.set_ylabel('theta (rad)')
ax3.set_title('Heading (theta) vs time')
ax3.grid(True)
ax3.legend()
theta_png = 'theta_vs_time_'+name+'.png'
fig3.savefig(theta_png, bbox_inches='tight', dpi=150)

# Prepare a small DataFrame with poses for display/save
pose_df = pd.DataFrame({
    'index': np.arange(N),
    'x_m': xs,
    'y_m': ys,
    'theta_rad': thetas,
    'left_deg': left_deg,
    'right_deg': right_deg
})

csv_path = 'trajectory_'+name+'.csv'
pose_df.to_csv(csv_path, index=False)

print(f"Saved files:\n - {traj_png}\n - {angles_png}\n - {csv_path}")

# Display the trajectory plot inline for the user
from IPython.display import Image, display
from pathlib import Path
display(Image(traj_png))
display(Image(angles_png))

# Also print the computed poses table
pose_df.head(20)


plt.show()