// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

/**
 * Extended Kalman Filter for robot pose estimation using wheel odometry and gyro.
 * 
 * State vector: [x, y, theta] where:
 *   - x, y: position in inches
 *   - theta: heading in radians
 * 
 * Prediction step uses differential drive odometry from encoder deltas.
 * Update step uses gyro heading measurements.
 */
public class EKFLocalizer {
  // Tuning constants
  /** Process noise for position (inches²) */
  private static final double PROCESS_NOISE_POSITION = 0.1;
  
  /** Process noise for heading (radians²) */
  private static final double PROCESS_NOISE_HEADING = 0.01;
  
  /** Measurement noise for gyro (radians²) */
  private static final double MEASUREMENT_NOISE_GYRO = 0.05;
  
  /** Track width between left and right wheels (inches) */
  private static final double TRACK_WIDTH = 6.0;

  // State: [x, y, theta]
  private double x;      // Position X (inches)
  private double y;      // Position Y (inches)
  private double theta;  // Heading (radians)

  // Covariance matrix (3x3)
  private Matrix3x3 P;

  // Process noise covariance (3x3)
  private Matrix3x3 Q;

  // Measurement noise covariance (scalar for gyro)
  private double R;

  // Previous encoder values for computing deltas
  private double prevLeftDistance;
  private double prevRightDistance;
  private boolean firstUpdate;

  /**
   * Creates a new EKF localizer with default initial state at origin.
   */
  public EKFLocalizer() {
    this(0.0, 0.0, 0.0);
  }

  /**
   * Creates a new EKF localizer with specified initial state.
   * @param initialX Initial X position (inches)
   * @param initialY Initial Y position (inches)
   * @param initialTheta Initial heading (radians)
   */
  public EKFLocalizer(double initialX, double initialY, double initialTheta) {
    this.x = initialX;
    this.y = initialY;
    this.theta = initialTheta;

    // Initialize covariance with small uncertainty
    P = Matrix3x3.identity().multiplyScalar(0.1);

    // Set up process noise covariance Q
    Q = new Matrix3x3(new double[][] {
      {PROCESS_NOISE_POSITION, 0, 0},
      {0, PROCESS_NOISE_POSITION, 0},
      {0, 0, PROCESS_NOISE_HEADING}
    });

    R = MEASUREMENT_NOISE_GYRO;

    prevLeftDistance = 0.0;
    prevRightDistance = 0.0;
    firstUpdate = true;
  }

  /**
   * Prediction step using wheel odometry.
   * Updates state estimate based on encoder measurements.
   * 
   * @param leftDistance Current left encoder distance (inches)
   * @param rightDistance Current right encoder distance (inches)
   */
  public void predict(double leftDistance, double rightDistance) {
    // On first call, just store the encoder values
    if (firstUpdate) {
      prevLeftDistance = leftDistance;
      prevRightDistance = rightDistance;
      firstUpdate = false;
      return;
    }

    // Compute encoder deltas
    double deltaLeft = leftDistance - prevLeftDistance;
    double deltaRight = rightDistance - prevRightDistance;
    
    // Store current values for next iteration
    prevLeftDistance = leftDistance;
    prevRightDistance = rightDistance;

    // Compute linear distance traveled and change in heading
    double distance = (deltaLeft + deltaRight) / 2.0;
    double dTheta = (deltaRight - deltaLeft) / TRACK_WIDTH;

    // Predict new state using motion model
    // x_new = x + distance * cos(theta)
    // y_new = y + distance * sin(theta)
    // theta_new = theta + dTheta
    double prevTheta = theta;
    x += distance * Math.cos(prevTheta);
    y += distance * Math.sin(prevTheta);
    theta += dTheta;

    // Normalize theta to [-pi, pi]
    theta = normalizeAngle(theta);

    // Compute Jacobian of motion model for covariance propagation
    // F = I + dg/dx where g is the motion model
    Matrix3x3 F = Matrix3x3.identity();
    F.set(0, 2, -distance * Math.sin(prevTheta));  // dx/dtheta
    F.set(1, 2, distance * Math.cos(prevTheta));   // dy/dtheta

    // Update covariance: P = F * P * F^T + Q
    Matrix3x3 Ft = F.transpose();
    P = F.multiply(P).multiply(Ft).add(Q);
  }

  /**
   * Update step using gyro measurement.
   * Corrects state estimate based on gyro heading.
   * 
   * @param gyroAngleDegrees Gyro Z-axis angle in degrees
   */
  public void update(double gyroAngleDegrees) {
    // Convert gyro measurement to radians
    double z = Math.toRadians(gyroAngleDegrees);
    z = normalizeAngle(z);

    // Measurement model: H = [0, 0, 1] (gyro directly observes heading)
    // Innovation: y = z - H*x = z - theta
    double innovation = normalizeAngle(z - theta);

    // Innovation covariance: S = H * P * H^T + R
    // Since H = [0, 0, 1], H*P*H^T = P[2][2]
    double S = P.get(2, 2) + R;

    // Kalman gain: K = P * H^T / S
    // K is a 3x1 vector: [P[0][2]/S, P[1][2]/S, P[2][2]/S]
    double k0 = P.get(0, 2) / S;
    double k1 = P.get(1, 2) / S;
    double k2 = P.get(2, 2) / S;

    // State update: x = x + K * innovation
    x += k0 * innovation;
    y += k1 * innovation;
    theta += k2 * innovation;
    theta = normalizeAngle(theta);

    // Covariance update: P = (I - K*H) * P
    // Since H = [0, 0, 1], K*H creates a matrix with K in the third column
    Matrix3x3 KH = new Matrix3x3();
    KH.set(0, 2, k0);
    KH.set(1, 2, k1);
    KH.set(2, 2, k2);
    
    Matrix3x3 I_minus_KH = Matrix3x3.identity().subtract(KH);
    P = I_minus_KH.multiply(P);
  }

  /**
   * Applies a correction to the state estimate.
   * Used for drift correction from map-based measurements.
   * 
   * @param dx Correction in X (inches)
   * @param dy Correction in Y (inches)
   */
  public void applyCorrection(double dx, double dy) {
    x += dx;
    y += dy;
  }

  /**
   * Normalizes an angle to the range [-π, π].
   * @param angle Angle in radians
   * @return Normalized angle
   */
  private double normalizeAngle(double angle) {
    while (angle > Math.PI) angle -= 2.0 * Math.PI;
    while (angle < -Math.PI) angle += 2.0 * Math.PI;
    return angle;
  }

  /**
   * Gets the current X position estimate.
   * @return X position in inches
   */
  public double getX() {
    return x;
  }

  /**
   * Gets the current Y position estimate.
   * @return Y position in inches
   */
  public double getY() {
    return y;
  }

  /**
   * Gets the current heading estimate.
   * @return Heading in radians
   */
  public double getTheta() {
    return theta;
  }

  /**
   * Gets the current heading estimate in degrees.
   * @return Heading in degrees
   */
  public double getThetaDegrees() {
    return Math.toDegrees(theta);
  }

  /**
   * Resets the localizer to a new pose.
   * @param newX New X position (inches)
   * @param newY New Y position (inches)
   * @param newTheta New heading (radians)
   */
  public void reset(double newX, double newY, double newTheta) {
    this.x = newX;
    this.y = newY;
    this.theta = normalizeAngle(newTheta);
    P = Matrix3x3.identity().multiplyScalar(0.1);
    firstUpdate = true;
  }
}
