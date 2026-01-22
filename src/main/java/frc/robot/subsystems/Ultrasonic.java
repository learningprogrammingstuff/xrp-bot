package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Ultrasonic extends SubsystemBase {
    private final XRPRangefinder m_Rangefinder;

    public Ultrasonic() {
        m_Rangefinder = new XRPRangefinder();
    }  
    public double getDistanceMeters() {
        return m_Rangefinder.getDistanceMeters();
    }

    public double getDistanceInches() {
        return m_Rangefinder.getDistanceInches();
    }

    @Override 
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Rangefinder/Distance (in)", getDistanceInches());
    }
}
