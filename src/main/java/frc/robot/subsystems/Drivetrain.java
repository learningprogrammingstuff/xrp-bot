// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm
  private static final double kWheelDiameterMeter = kWheelDiameterInch * 0.0254; // Convert to meters
  private static final double kTrackWidthMeter = 0.155; // XRP track width ~155 mm
  private static final double kMaxSpeedMetersPerSecond = 0.5; // XRP max speed ~0.5 m/s

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // Kinematics and odometry for PathPlanner
  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidthMeter);
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use meters as unit for encoder distances (PathPlanner requires meters)
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    resetEncoders();

    // Initialize odometry at the origin
    m_odometry = new DifferentialDriveOdometry(
        Rotation2d.fromDegrees(getGyroAngleZ()),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Only configure AutoBuilder if config loaded successfully
    if (config != null) {
      AutoBuilder.configure(
              this::getPose,
              this::resetPose,
              this::getRobotRelativeSpeeds,
              (speeds, feedforwards) -> driveRobotRelative(speeds),
              new PPLTVController(0.02),
              config,
              () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this
      );
    }
  }
  

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  /** Drive the robot using robot-relative ChassisSpeeds (used by PathPlanner). */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    // Normalize wheel speeds to [-1, 1] range
    double leftOutput = wheelSpeeds.leftMetersPerSecond / kMaxSpeedMetersPerSecond;
    double rightOutput = wheelSpeeds.rightMetersPerSecond / kMaxSpeedMetersPerSecond;
    // Clamp to [-1, 1]
    leftOutput = Math.max(-1.0, Math.min(1.0, leftOutput));
    rightOutput = Math.max(-1.0, Math.min(1.0, rightOutput));
    m_leftMotor.set(leftOutput);
    m_rightMotor.set(rightOutput);
  }

  /** Get the current robot pose from odometry. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /** Reset the odometry to a given pose. */
  public void resetPose(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getGyroAngleZ()),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance(),
        pose);
  }

  /** Get robot-relative ChassisSpeeds (used by PathPlanner). */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            m_leftEncoder.getRate(),
            m_rightEncoder.getRate()));
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance() / 0.0254;
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance() / 0.0254;
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the XRP along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the XRP along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the XRP along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the XRP around the X-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the XRP around the Y-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the XRP around the Z-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // Update odometry each scheduler run
    m_odometry.update(
        Rotation2d.fromDegrees(getGyroAngleZ()),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());
  }
}