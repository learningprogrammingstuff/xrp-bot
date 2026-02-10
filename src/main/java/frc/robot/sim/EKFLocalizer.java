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
 * 
 * Includes wheel slip detection: when encoder-derived yaw rate disagrees with
 * the gyro, encoder trust is reduced (process noise inflated) until the
 * disagreement subsides.
 */
public class EKFLocalizer {
  // Tuning constants
  /** Base process noise for position (inches²) */
  private static final double PROCESS_NOISE_POSITION = 0.1;
  
  /** Base process noise for heading (radians²) */
  private static final double PROCESS_NOISE_HEADING = 0.01;
  
  /** Measurement noise for gyro (radians²) */
  private static final double MEASUREMENT_NOISE_GYRO = 0.05;
  
  /** Track width between left and right wheels (inches) */
  private static final double TRACK_WIDTH = 6.0;

  // Slip detection tuning
  /** Yaw rate disagreement threshold for slip detection (radians per tick) */
  private static final double SLIP_YAW_RATE_THRESHOLD = 0.08;

  /** Consecutive frames required to enter slip mode */
  private static final int SLIP_ENTER_FRAMES = 3;

  /** Consecutive non-slip frames required to exit slip mode */
  private static final int SLIP_EXIT_FRAMES = 5;

  /** Multiplier for process noise during detected slip */
  private static final double SLIP_NOISE_MULTIPLIER = 10.0;

  // State: [x, y, theta]
  private double x;      // Position X (inches)
  private double y;      // Position Y (inches)
  private double theta;  // Heading (radians)

  // Covariance matrix (3x3)
  private Matrix3x3 P;

  // Process noise covariance (3x3) – base values, inflated during slip
  private Matrix3x3 Q;

  // Measurement noise covariance (scalar for gyro)
  private double R;

  // Previous encoder values for computing deltas
  private double prevLeftDistance;
  private double prevRightDistance;
  private boolean firstUpdate;

  // Slip detection state
  private boolean slipDetected;
  private int slipFrameCounter;
  private int nonSlipFrameCounter;
  private double prevGyroAngleRad;
  private boolean gyroInitialized;
  private double lastEncoderDTheta;

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

    // Slip detection state
    slipDetected = false;
    slipFrameCounter = 0;
    nonSlipFrameCounter = 0;
    prevGyroAngleRad = 0.0;
    gyroInitialized = false;
    lastEncoderDTheta = 0.0;
  }

  /**
   * Prediction step using wheel odometry.
   * Updates state estimate based on encoder measurements.
   * When wheel slip is detected, encoder process noise is inflated
   * so the gyro correction dominates.
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

    // Store encoder-derived yaw change for slip detection
    lastEncoderDTheta = dTheta;

    // Predict new state using motion model
    double prevTheta = theta;
    x += distance * Math.cos(prevTheta);
    y += distance * Math.sin(prevTheta);
    theta += dTheta;

    // Normalize theta to [-pi, pi]
    theta = normalizeAngle(theta);

    // Compute Jacobian of motion model for covariance propagation
    Matrix3x3 F = Matrix3x3.identity();
    F.set(0, 2, -distance * Math.sin(prevTheta));  // dx/dtheta
    F.set(1, 2, distance * Math.cos(prevTheta));   // dy/dtheta

    // Apply adaptive process noise – inflate when slip is detected
    Matrix3x3 Qeff = Q;
    if (slipDetected) {
      Qeff = new Matrix3x3(new double[][] {
        {PROCESS_NOISE_POSITION * SLIP_NOISE_MULTIPLIER, 0, 0},
        {0, PROCESS_NOISE_POSITION * SLIP_NOISE_MULTIPLIER, 0},
        {0, 0, PROCESS_NOISE_HEADING}
      });
    }

    // Update covariance: P = F * P * F^T + Q_eff
    Matrix3x3 Ft = F.transpose();
    P = F.multiply(P).multiply(Ft).add(Qeff);
  }

  /**
   * Update step using gyro measurement.
   * Corrects state estimate based on gyro heading.
   * Also runs wheel slip detection by comparing encoder-derived yaw rate
   * against gyro-derived yaw rate.
   * 
   * @param gyroAngleDegrees Gyro Z-axis angle in degrees
   */
  public void update(double gyroAngleDegrees) {
    // Convert gyro measurement to radians
    double z = Math.toRadians(gyroAngleDegrees);
    z = normalizeAngle(z);

    // Slip detection: compare incremental encoder yaw change vs gyro yaw change
    if (gyroInitialized) {
      double gyroYawDelta = normalizeAngle(z - prevGyroAngleRad);
      // Compare encoder-derived yaw change (from predict step) vs gyro-derived yaw change
      double encoderVsGyroDisagreement = Math.abs(normalizeAngle(lastEncoderDTheta - gyroYawDelta));

      if (encoderVsGyroDisagreement > SLIP_YAW_RATE_THRESHOLD) {
        slipFrameCounter++;
        nonSlipFrameCounter = 0;
        if (slipFrameCounter >= SLIP_ENTER_FRAMES && !slipDetected) {
          slipDetected = true;
        }
      } else {
        nonSlipFrameCounter++;
        slipFrameCounter = 0;
        if (nonSlipFrameCounter >= SLIP_EXIT_FRAMES && slipDetected) {
          slipDetected = false;
        }
      }
    }
    prevGyroAngleRad = z;
    gyroInitialized = true;

    // Measurement model: H = [0, 0, 1] (gyro directly observes heading)
    double innovation = normalizeAngle(z - theta);

    // Innovation covariance: S = H * P * H^T + R
    double S = P.get(2, 2) + R;

    // Kalman gain: K = P * H^T / S
    double k0 = P.get(0, 2) / S;
    double k1 = P.get(1, 2) / S;
    double k2 = P.get(2, 2) / S;

    // State update: x = x + K * innovation
    x += k0 * innovation;
    y += k1 * innovation;
    theta += k2 * innovation;
    theta = normalizeAngle(theta);

    // Covariance update: P = (I - K*H) * P
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
    slipDetected = false;
    slipFrameCounter = 0;
    nonSlipFrameCounter = 0;
    gyroInitialized = false;
    lastEncoderDTheta = 0.0;
  }

  /**
   * Returns whether wheel slip is currently detected.
   * @return true if slip mode is active
   */
  public boolean isSlipDetected() {
    return slipDetected;
  }
}
