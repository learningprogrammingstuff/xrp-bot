package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveUntilDistance extends Command {
  private final Drivetrain m_drive;
  private final XRPRangefinder m_rangefinder;
  private final double m_speed;
  private final double m_targetDistanceInches;

  /**
   * Drives forward until the rangefinder detects an obstacle within the target distance.
   *
   * @param speed The speed at which the robot will drive (-1.0 to 1.0)
   * @param targetDistanceInches The distance in inches to stop at
   * @param drive The drivetrain subsystem
   * @param rangefinder The XRPRangefinder instance
   */
  public DriveUntilDistance(double speed, double targetDistanceInches, 
                            Drivetrain drive, XRPRangefinder rangefinder) {
    m_speed = speed;
    m_targetDistanceInches = targetDistanceInches;
    m_drive = drive;
    m_rangefinder = rangefinder;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentDistance = m_rangefinder.getDistanceInches();
    
    // Stop when we're within the target distance
    // Also stop if we get an invalid reading (> 157 inches means no obstacle detected)
    return currentDistance <= m_targetDistanceInches && currentDistance < 157.0;
  }
}