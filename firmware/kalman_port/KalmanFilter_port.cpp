// KalmanFilter_port.cpp
// My 2-state Kalman filter (angle + gyro bias), ported from python/analyze_tilt.py.
// This function REPLACES KalmanFilter::Angle() in ELEGOO's KalmanFilter.cpp. Everything
// else in that file is ELEGOO's stock code and is not included here.
// It needs the kf_* members from KalmanFilter_port.h added to KalmanFilter.h.
// ELEGOO source: https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial

void KalmanFilter::Angle(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1) {
  // The Q_angle, Q_gyro, R_angle, C_0, K1 arguments are ignored on purpose;
  // this filter uses its own constants (kf_q_angle, kf_q_bias, kf_r in KalmanFilter.h).

  float z = atan2(ay, az) * 57.3;   // Python: accel_angle_deg = arctan2(ay, az) in degrees (57.3 = 180/pi, rounded)
  float w = gx / 131.0;             // Python: raw_gyro_rate_dps = gx / GYRO_SCALE (131.0); no 128.1 offset

  if (!kf_started) {                // Python: if not self.started:
    angle = z;                      //   self.angle = accel_angle_deg
    kf_started = true;              //   self.started = True
  } else {                          // (Python returns here on the first call; the else just lets Gyro_x/Gyro_z below still run)

    // ---- Predict state ----
    angle += (w - kf_bias) * dt;    // Python: self.angle += gyro_rate_dps*dt - self.bias*dt   (bias unchanged)

    // ---- Predict covariance: P = F*P*F^T + Q, F = [[1,-dt],[0,1]] ----
    // Use the OLD p values for all four results, so store them in temporaries first.
    float n00 = kf_p00 - dt * (kf_p01 + kf_p10) + dt * dt * kf_p11 + kf_q_angle * dt;  // Python: (F@P@F.T + Q)[0,0]
    float n01 = kf_p01 - dt * kf_p11;                                                   // Python: (F@P@F.T)[0,1]
    float n10 = kf_p10 - dt * kf_p11;                                                   // Python: (F@P@F.T)[1,0]
    float n11 = kf_p11 + kf_q_bias * dt;                                                // Python: (F@P@F.T + Q)[1,1]
    kf_p00 = n00;                   // Python: self.P = F @ self.P @ F.T + Q
    kf_p01 = n01;
    kf_p10 = n10;
    kf_p11 = n11;

    // ---- Update ----
    float y = z - angle;            // Python: y = accel_angle_deg - self.angle   (computed once)
    float S = kf_p00 + kf_r;        // Python: S = self.P[0,0] + self.r
    float K0 = kf_p00 / S;          // Python: K = self.P[:,0] / S  -> K[0]
    float K1g = kf_p10 / S;         // Python: K[1]  (named K1g because K1 is already a function argument name)
    angle += K0 * y;                // Python: self.angle += K[0]*y
    kf_bias += K1g * y;             // Python: self.bias += K[1]*y

    // ---- Covariance update: P = P - outer(K, P[0,:]) ----
    // Uses the pre-update p00 and p01, so again store in temporaries first.
    float m00 = (1 - K0) * kf_p00;  // Python: (P - outer(K, P[0,:]))[0,0] = p00 - K0*p00
    float m01 = (1 - K0) * kf_p01;  // Python: [0,1] = p01 - K0*p01
    float m10 = kf_p10 - K1g * kf_p00;  // Python: [1,0] = p10 - K1*p00
    float m11 = kf_p11 - K1g * kf_p01;  // Python: [1,1] = p11 - K1*p01
    kf_p00 = m00;                   // Python: self.P = self.P - np.outer(K, self.P[0,:])
    kf_p01 = m01;
    kf_p10 = m10;
    kf_p11 = m11;
  }

  Gyro_x = w - kf_bias;             // Output for the controller's D term: bias-corrected rate (w - bias)
  Gyro_z = -gz / 131;               // Unchanged from ELEGOO stock
}
