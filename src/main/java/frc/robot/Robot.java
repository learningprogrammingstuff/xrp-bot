// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.EKFLocalizer;
import frc.robot.sim.OccupancyMapper;
import frc.robot.sim.RangeFilter;
import frc.robot.sim.SimWorld;
import frc.robot.sim.VisualizationServer;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // Simulation components
  private EKFLocalizer ekfLocalizer;
  private OccupancyMapper mapper;
  private SimWorld simWorld;
  private VisualizationServer vizServer;
  private AnalogInputSim rangefinderSim;
  private RangeFilter rangeFilter;

  /**
   * XRP rangefinder maximum distance (meters). The hardware sensor saturates at
   * 4.0 m; voltages above the corresponding level are clamped.
   */
  private static final double XRP_MAX_RANGE_M = 4.0;

  /**
   * XRP rangefinder AnalogInput full-scale voltage. The sensor maps 0–5 V to
   * the full distance range.
   */
  private static final double XRP_FULL_SCALE_VOLTAGE = 5.0;

  /** Parity tolerance: maximum acceptable difference between AnalogInput-derived
   *  and subsystem-reported distance before logging a warning (meters). */
  private static final double PARITY_TOLERANCE_M = 0.01;

  /** Simulation cycle counter for debug logging. */
  private long simCycleCount;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    try {
      // Initialize simulation components
      ekfLocalizer = new EKFLocalizer(60.0, 48.0, 0.0); // Start at center of room

      // Load ground-truth world from external file (robot code never reads this file).
      // Falls back to procedural generation if file is missing.
      simWorld = new SimWorld("world.json");

      // Mapper starts with an empty/unknown grid – it discovers the world via
      // sensor readings only.  The grid dimensions match the room purely for
      // sizing; no obstacle information is shared.
      mapper = new OccupancyMapper("xrp-map.json", simWorld.getRoomWidth(), simWorld.getRoomHeight());

      vizServer = new VisualizationServer();
      
      // Create simulation handle for the rangefinder's analog input (channel 2)
      rangefinderSim = new AnalogInputSim(2);

      // Range filter: 5-sample median + outlier rejection
      rangeFilter = new RangeFilter();
      simCycleCount = 0;

      // Configure visualization server
      vizServer.setSimWorld(simWorld);
      vizServer.setMapper(mapper);
      vizServer.setLocalizer(ekfLocalizer);
      
      // Start server and open browser
      vizServer.start();
      
      System.out.println("Simulation initialized with 3D visualization");
    } catch (Exception e) {
      System.err.println("Failed to initialize simulation: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    if (ekfLocalizer == null || mapper == null || simWorld == null || vizServer == null) {
      return;
    }

    try {
      simCycleCount++;

      // 1. Read encoder values and gyro from drivetrain
      var drivetrain = m_robotContainer.getDrivetrain();
      double leftDistance = drivetrain.getLeftDistanceInch();
      double rightDistance = drivetrain.getRightDistanceInch();
      double gyroAngleZ = drivetrain.getGyroAngleZ();

      // 2. Run EKF prediction (odometry) then update (gyro)
      ekfLocalizer.predict(leftDistance, rightDistance);
      ekfLocalizer.update(gyroAngleZ);

      // 3. Get current pose from EKF
      double robotX = ekfLocalizer.getX();
      double robotY = ekfLocalizer.getY();
      double robotTheta = ekfLocalizer.getTheta();

      // 4. Simulate ultrasonic reading using current pose (inches, unclamped)
      double sensorOffset = 2.0; // inches
      double simulatedRangeInches = simWorld.simulateUltrasonicReading(
          robotX, robotY, robotTheta, sensorOffset);

      // 4a. Convert to meters (unclamped raycast distance for mapping)
      double rawRangeMeters = Units.inchesToMeters(simulatedRangeInches);

      // 4b. Clamp to XRP hardware range (0–4 m) and compute the AnalogInput voltage.
      //     XRPRangefinder formula: distanceMeters = (voltage / 5.0) * 4.0
      //     Inverse:                voltage         = distanceMeters * (5.0 / 4.0)
      double clampedMeters = Math.min(rawRangeMeters, XRP_MAX_RANGE_M);
      double voltage = clampedMeters * (XRP_FULL_SCALE_VOLTAGE / XRP_MAX_RANGE_M);
      voltage = Math.max(0.0, Math.min(XRP_FULL_SCALE_VOLTAGE, voltage));
      rangefinderSim.setVoltage(voltage);

      // 4c. Read back through the subsystem (same path SimUI uses)
      double subsystemMeters = m_robotContainer.getUltrasonic().getDistanceMeters();

      // 4d. SimUI parity check: compare AnalogInput-derived distance to subsystem
      double aiDerivedMeters = (voltage / XRP_FULL_SCALE_VOLTAGE) * XRP_MAX_RANGE_M;
      double parityDelta = Math.abs(aiDerivedMeters - subsystemMeters);
      if (parityDelta > PARITY_TOLERANCE_M) {
        System.err.printf("[Cycle %d] PARITY MISMATCH: AI-derived=%.4f m, "
            + "subsystem=%.4f m, delta=%.4f m%n",
            simCycleCount, aiDerivedMeters, subsystemMeters, parityDelta);
      }

      // 4e. Apply range filter to the clamped distance for mapping/visualization
      double filteredMeters = rangeFilter.filter(clampedMeters);
      double filteredInches = Units.metersToInches(filteredMeters);
      boolean validReading = rangeFilter.hasValidMeasurement();

      // 4f. Debug output: log all values every 50 cycles (~1 second)
      // AnalogInput raw counts: 12-bit ADC over 0–5V → counts = voltage * 4095 / 5.0
      int rawCounts = (int) Math.round(voltage * 4095.0 / XRP_FULL_SCALE_VOLTAGE);
      if (simCycleCount % 50 == 1) {
        System.out.printf("[Cycle %d] AI2: counts=%d voltage=%.3fV | "
            + "clamped=%.3fm raw=%.3fm filtered=%.3fm | valid=%b%n",
            simCycleCount, rawCounts, voltage,
            clampedMeters, rawRangeMeters, filteredMeters, validReading);
      }

      // 5. Add map point using FILTERED + CLAMPED range (inches)
      mapper.addPoint(filteredInches, robotX, robotY, robotTheta);

      // 6. Check for drift correction
      double[] correction = mapper.checkDriftCorrection(
          filteredInches, robotX, robotY, robotTheta);
      if (correction != null) {
        ekfLocalizer.applyCorrection(correction[0], correction[1]);
      }

      // 7. Update visualization server with latest state
      vizServer.updatePose(robotX, robotY, robotTheta);
      vizServer.updateMap(mapper.getMapPoints());
      vizServer.updateBeam(filteredInches, robotX, robotY, robotTheta);
      vizServer.updateDistances(drivetrain.getAverageDistanceInch(), filteredInches);
      vizServer.updateRangefinderDebug(
          rawCounts,             // raw counts
          voltage,               // voltage
          clampedMeters,         // clamped meters
          rawRangeMeters,        // raw/unclamped meters
          filteredMeters,        // filtered meters
          validReading           // valid flag
      );

    } catch (Exception e) {
      System.err.println("Error in simulation periodic: " + e.getMessage());
    }
  }
}
