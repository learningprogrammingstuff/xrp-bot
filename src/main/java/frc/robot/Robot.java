// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.EKFLocalizer;
import frc.robot.sim.OccupancyMapper;
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
      mapper = new OccupancyMapper();
      simWorld = new SimWorld();
      vizServer = new VisualizationServer();
      
      // Configure visualization server
      vizServer.setSimWorld(simWorld);
      
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

      // 4. Simulate ultrasonic reading using current pose
      double sensorOffset = 2.0; // inches
      double simulatedRange = simWorld.simulateUltrasonicReading(robotX, robotY, robotTheta, sensorOffset);

      // 5. Add map point if valid
      mapper.addPoint(simulatedRange, robotX, robotY, robotTheta);

      // 6. Check for drift correction
      double[] correction = mapper.checkDriftCorrection(simulatedRange, robotX, robotY, robotTheta);
      if (correction != null) {
        ekfLocalizer.applyCorrection(correction[0], correction[1]);
      }

      // 7. Update visualization server with latest state
      vizServer.updatePose(robotX, robotY, robotTheta);
      vizServer.updateMap(mapper.getMapPoints());
      vizServer.updateBeam(simulatedRange, robotX, robotY, robotTheta);

    } catch (Exception e) {
      System.err.println("Error in simulation periodic: " + e.getMessage());
    }
  }
}
