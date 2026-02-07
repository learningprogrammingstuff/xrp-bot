// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain, XRPRangefinder rangefinder) {
    addCommands(
        // new DriveDistance(1, 1, drivetrain),
        new DriveUntilDistance(1, 14, drivetrain, rangefinder),
        new WaitCommand(0.5),
        new TurnDegrees(0.8, ((Math.random() * 70) + 20), drivetrain),
        new WaitCommand(0.5),
                new DriveUntilDistance(1, 14, drivetrain, rangefinder),
        new WaitCommand(0.5),
        new TurnDegrees(0.8, ((Math.random() * 70) + 20), drivetrain),
        new WaitCommand(0.5),
                new DriveUntilDistance(1, 14, drivetrain, rangefinder),
        new WaitCommand(0.5),
        new TurnDegrees(0.8, ((Math.random() * 70) + 20), drivetrain),
        new WaitCommand(0.5),
                new DriveUntilDistance(1, 14, drivetrain, rangefinder),
        new WaitCommand(0.5),
        new TurnDegrees(0.8, ((Math.random() * 70) + 20), drivetrain),
        new WaitCommand(0.5),
                new DriveUntilDistance(1, 14, drivetrain, rangefinder),
        new WaitCommand(0.5),
        new TurnDegrees(0.8, ((Math.random() * 70) + 20), drivetrain),
        new WaitCommand(0.5),
                new DriveUntilDistance(1, 14, drivetrain, rangefinder),
        new WaitCommand(0.5),
        new TurnDegrees(0.8, ((Math.random() * 70) + 20), drivetrain),
        new WaitCommand(0.5),
        new DriveUntilDistance(1, 14, drivetrain, rangefinder),
        new WaitCommand(0.5),
        new TurnDegrees(0.8, ((Math.random() * 70) + 20), drivetrain),
        new WaitCommand(0.5),
        new DriveUntilDistance(1, 14, drivetrain, rangefinder),
        new WaitCommand(0.5),
        new TurnDegrees(0.8, ((Math.random() * 70) + 20), drivetrain),
        new WaitCommand(0.5),
        new DriveUntilDistance(1, 14, drivetrain, rangefinder),
        new WaitCommand(0.5),
        new TurnDegrees(0.8, ((Math.random() * 70) + 20), drivetrain),
        new WaitCommand(0.5),
        new DriveUntilDistance(1, 14, drivetrain, rangefinder),
        new WaitCommand(0.5),
        new TurnDegrees(0.8, ((Math.random() * 70) + 20), drivetrain),
        new WaitCommand(0.5));
  }
}
