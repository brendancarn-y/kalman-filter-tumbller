// KalmanFilter_port.h
// My 2-state Kalman filter (angle + gyro bias), ported from python/analyze_tilt.py.
// These lines REPLACE/ADD TO the private section of ELEGOO's KalmanFilter.h: paste them
// just before the closing "};" of class KalmanFilter. Everything else in that file is
// ELEGOO's stock code and is not included here.
// ELEGOO source: https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial

  // --- My TwoStateKalman filter (ported from analyze_tilt.py) ---
  // Names start with kf_ so they don't clash with the stock q_bias/P/angle_err above.
  bool  kf_started = false;     // Python: self.started = False
  float kf_bias = 1.898;        // Python: self.bias = 1.898 (deg/s)
  float kf_p00 = 1.0, kf_p01 = 0.0, kf_p10 = 0.0, kf_p11 = 0.25;  // Python: self.P = [[1,0],[0,0.25]]
  float kf_q_angle = 0.001;     // Python: self.q_angle = 0.001
  float kf_q_bias = 0.0003;     // Python: self.q_bias = 0.0003
  float kf_r = 0.5;             // Python: self.r = 0.5
