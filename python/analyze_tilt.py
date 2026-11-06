"""analyze_tilt.py: plot the accel-only angle, the pure gyro-integrated angle,
and your Kalman filter's angle estimate, all on one graph, so you can see
whether your filter is doing its job: smooth like gyro, but not drifting off
like plain gyro-integration does.

Fill in TwoStateKalman.step() below with your own filter math, then run:
    python analyze_tilt.py tilt_test.csv
Add --save out.png instead of popping up a window (useful for quick checks).
"""
import argparse
import csv

import matplotlib.pyplot as plt
import numpy as np

# Scale factors matching the sketch's sensor settings (+-2g, +-250 deg/s).
ACCEL_SCALE = 16384.0  # counts per g
GYRO_SCALE = 131.0     # counts per deg/s


class TwoStateKalman:
    """2-state Kalman filter: state = [angle, gyro_bias].

    This is yours to write. step() below is just the per-sample loop driver
    that calls your predict/update logic and returns the current angle.
    """

    def __init__(self):
        self.angle = 0.0
        self.bias = 1.898
        # TODO: initialize your covariance matrix / process & measurement noise here
        self.P = np.array([[1, 0], [0, 0.25]])
        self.q_angle = 0.001
        self.q_bias = 0.0003
        self.r = 0.5
        self.started = False

    def step(self, accel_angle_deg, gyro_rate_dps, dt):
        """Called once per sample in time order. Must return the angle estimate."""
        if not self.started:
            self.angle = accel_angle_deg
            self.started = True
            return self.angle
        # TODO: predict step   (advance self.angle using gyro_rate_dps, dt, self.bias)
        self.angle += gyro_rate_dps * dt - self.bias * dt
        F = np.array([[1, -dt], [0, 1]])
        Q = np.array([[self.q_angle*dt,0],[0,self.q_bias*dt]])
        self.P = F @ self.P @ F.T + Q
        # TODO: update step    (correct self.angle, self.bias using accel_angle_deg)
        y = accel_angle_deg - self.angle
        S = self.P[0,0] + self.r
        K = self.P[:,0] / S
        self.angle += K[0]*y
        self.bias += K[1]*y
        self.P = self.P - np.outer(K,self.P[0,:])
        return self.angle


def load_csv(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    data = np.array(rows[1:], dtype=float)
    t_us, ax, ay, az, gx, gy, gz = data.T
    return t_us, ax, ay, az, gx, gy, gz


def main():
    p = argparse.ArgumentParser()
    p.add_argument("csv_path")
    p.add_argument("--save", help="save plot to this file instead of showing a window")
    a = p.parse_args()

    t_us, ax, ay, az, gx, gy, gz = load_csv(a.csv_path)
    t = (t_us - t_us[0]) / 1e6  # seconds, starting at 0

    # Tilt axis note: in the recorded test, ax barely moved while ay/az swung
    # hugely, so the robot's pitch (balance) axis rotates in the Y-Z plane,
    # and the matching gyro channel is gx. If your mounting differs, these
    # two lines are the only thing you'd need to change.
    accel_angle_deg = np.degrees(np.arctan2(ay, az))
    raw_gyro_rate_dps = gx / GYRO_SCALE

    dt = np.diff(t, prepend=t[0])
    dt[0] = dt[1]  # first sample has no real dt behind it; reuse the second

    # --- sample-timing sanity check ---
    print(f"dt (s): min={dt.min():.6f}  median={np.median(dt):.6f}  max={dt.max():.6f}")

    # --- raw gyro bias, first vs. last 3s (robot should be ~still at the start) ---
    first3 = t < 3.0
    last3 = t > (t[-1] - 3.0)
    print(f"gyro bias (gx/131), first 3s: {raw_gyro_rate_dps[first3].mean():+.4f} deg/s")
    print(f"gyro bias (gx/131), last 3s:  {raw_gyro_rate_dps[last3].mean():+.4f} deg/s")

    # --- does positive gyro rate mean the same direction as increasing accel angle? ---
    accel_rate_dps = np.gradient(accel_angle_deg, t)
    corr = np.corrcoef(accel_rate_dps, raw_gyro_rate_dps)[0, 1]
    print(f"accel-rate vs raw gyro-rate correlation: {corr:+.3f}", end="  ")
    if corr < 0:
        print("-> signs disagree, flipping gyro_rate_dps")
        gyro_rate_dps = -raw_gyro_rate_dps
    else:
        print("-> signs agree, no flip needed")
        gyro_rate_dps = raw_gyro_rate_dps

    # --- initial angle from the first 0.5s (robot held still) ---
    theta0 = accel_angle_deg[t < 0.5].mean()
    print(f"theta0 (initial angle, mean accel angle over first 0.5s): {theta0:.3f} deg")

    gyro_integrated_deg = theta0 + np.cumsum(gyro_rate_dps * dt)

    kf = TwoStateKalman()
    kf.angle = theta0  # start the filter from the same measured initial tilt, not 0
    kf_angle_deg = np.zeros_like(t)
    kf_bias_deg = np.zeros_like(t)
    have_kf = True
    for i in range(len(t)):
        try:
            kf_angle_deg[i] = kf.step(accel_angle_deg[i], gyro_rate_dps[i], dt[i])
            kf_bias_deg[i] = kf.bias
        except NotImplementedError:
            have_kf = False
            break

    if not have_kf:
        print("TwoStateKalman.step() isn't written yet -- plotting the two "
              "reference signals only. Fill it in and re-run to add your line.")

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(11, 8), sharex=True, gridspec_kw={"height_ratios": [3, 1]}
    )
    ax1.plot(t, accel_angle_deg, label="accel angle (noisy)", alpha=0.5)
    ax1.plot(t, gyro_integrated_deg, label="gyro integrated (drifts)", alpha=0.7)
    if have_kf:
        ax1.plot(t, kf_angle_deg, label="your Kalman filter", linewidth=2)
    ax1.set_ylabel("angle (deg)")
    ax1.set_title("Tilt angle: accel vs gyro-integration vs your filter")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    if have_kf:
        ax2.plot(t, kf_bias_deg, color="tab:green")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("bias (deg/s)")
    ax2.set_title("filter's estimated gyro bias")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    if a.save:
        plt.savefig(a.save, dpi=120)
        print(f"Saved plot to {a.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
