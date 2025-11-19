#!/usr/bin/env python3
"""
compute_odometry_from_data.py

- Lit un fichier data (format Matlab / ta data text file)
- Applique le preprocessing (skip motionless, subsample, quantize/round/dumbFactor)
- Convertit ticks -> radians puis calcule l'odométrie (x,y,theta)
- Sauve un CSV et trace la trajectoire

Usage:
    python3 compute_odometry_from_data.py path/to/datafile.txt
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import math

def preprocess_data(data, dots2rad, dumbFactor, subSamplingFactor):
    # data: numpy array shape (Ncols)
    nbSamples = data.shape[0]
    # skip initial motionless
    i = 0
    while (i < nbSamples-1) and (data[i,0] - data[i+1,0] == 0) and (data[i,1] - data[i+1,1] == 0):
        i += 1
    startIndex = i

    # skip final motionless
    i = nbSamples - 1
    while (i > 0) and (data[i,0] - data[i-1,0] == 0) and (data[i,1] - data[i-1,1] == 0):
        i -= 1
    stopIndex = i

    # slice and oneOfN (subsampling)
    sliced = data[startIndex:stopIndex+1, :]
    if subSamplingFactor > 1:
        sliced = sliced[::subSamplingFactor, :]

    # convert to rad, applying dumbFactor quantization exactly like Matlab:
    # round(col/dumbFactor) * dots2rad
    qL = np.round(sliced[:,0] / dumbFactor) * dots2rad
    qR = np.round(sliced[:,1] / dumbFactor) * dots2rad
    t  = sliced[:,3] - sliced[0,3]   # time relative to first sample
    sensorReadings = sliced[:,2].astype(int)

    return t, qL, qR, sensorReadings

def integrate_odometry(qL, qR, r, L, x0=(0.0,0.0,0.0)):
    N = qL.size
    xs = np.zeros(N)
    ys = np.zeros(N)
    thetas = np.zeros(N)
    x, y, theta = x0
    xs[0], ys[0], thetas[0] = x, y, theta

    for i in range(1, N):
        deltaqL = qL[i] - qL[i-1]
        deltaqR = qR[i] - qR[i-1]
        # linear displacements
        dL = r * deltaqL
        dR = r * deltaqR
        delta_d = 0.5 * (dL + dR)
        delta_theta = (dR - dL) / L

        # integrate
        x = x + delta_d * math.cos(theta)
        y = y + delta_d * math.sin(theta)
        theta = theta + delta_theta
        # normalize theta
        theta = math.atan2(math.sin(theta), math.cos(theta))

        xs[i], ys[i], thetas[i] = x, y, theta

    return xs, ys, thetas

def main(filepath):
    # Robot parameters (take from Matlab defaults, convert mm->m)
    rwheel_mm = 21.5
    trackGauge_mm = 112.0
    actualEncoderRes = 360.0
    dumbFactor = 8.0
    subSamplingFactor = 4

    # convert to meters/radians
    r = rwheel_mm / 1000.0
    L = trackGauge_mm / 1000.0
    encoderRes = actualEncoderRes / dumbFactor  # effective encoder res after dumbFactor
    dots2rad = (2.0 * math.pi) / encoderRes

    # load file (whitespace separated)
    df = pd.read_csv(filepath, sep=r'\s+', header=None, engine='python')
    data = df.values  # numpy array

    t, qL, qR, sensorReadings = preprocess_data(data, dots2rad, dumbFactor, subSamplingFactor)

    xs, ys, thetas = integrate_odometry(qL, qR, r, L, x0=(0.0, 0.0, 0.0))

    # Save results
    outdir = Path(filepath).with_suffix('')
    outdir = Path(str(outdir) + '_odometry')
    outdir.mkdir(parents=True, exist_ok=True)
    results_df = pd.DataFrame({
        't': t,
        'qL_rad': qL,
        'qR_rad': qR,
        'x_m': xs,
        'y_m': ys,
        'theta_rad': thetas,
        'sensor_raw': sensorReadings
    })
    csv_out = outdir / 'odometry_results.csv'
    results_df.to_csv(csv_out, index=False)

    # Plots
    plt.figure(figsize=(6,6))
    plt.plot(xs, ys, '-o')
    plt.axis('equal')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Trajectory from odometry')
    plt.grid(True)
    plt.savefig(outdir / 'trajectory.png', dpi=150)

    plt.figure()
    plt.plot(t, np.rad2deg(qL), label='qL (deg)')
    plt.plot(t, np.rad2deg(qR), label='qR (deg)')
    plt.xlabel('time [s]')
    plt.ylabel('wheel angle [deg]')
    plt.legend()
    plt.savefig(outdir / 'wheel_angles.png', dpi=150)

    print(f"Saved results in {outdir}")

if __name__ == '__main__':
    
    main("line1magnet.txt")
