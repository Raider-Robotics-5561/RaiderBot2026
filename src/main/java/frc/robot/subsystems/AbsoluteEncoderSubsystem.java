package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AbsoluteEncoderSubsystem extends SubsystemBase {
    // Initialize on a PWM port (e.g., PWM 0)
    private final DutyCycleEncoder m_angleEncoder = new DutyCycleEncoder(9);
    private final double abs_encoder_offset = 0.856;

    public AbsoluteEncoderSubsystem() {
        // Set the range for 0-360 degrees (0 to 1 scaling)
        m_angleEncoder.setDutyCycleRange(1.0/1024.0, 1023.0/1024.0);
        // Optional: Set distance per rotation (e.g., 360 degrees)
    }

    @Override
    public void periodic() {
        // Get angle in degrees (0–360)
        SmartDashboard.putNumber("Absolute Angle Raw", getRawAngle());
        SmartDashboard.putNumber("Absolute Angle With Offset", getAngle());
        SmartDashboard.putNumber("Absolute Angle Degrees", getAngleDegrees());
    }

    public DutyCycleEncoder getDutyCycleEncoder() {
        return m_angleEncoder;
    }

    public double getRawAngle() {
        return m_angleEncoder.get();
    }

    public double getAngle() {
        double raw = m_angleEncoder.get();
        // Apply offset and wrap into [0, 1)
        double adjusted = raw - abs_encoder_offset;
        adjusted = adjusted - Math.floor(adjusted); // normalizes to [0,1)
        return adjusted;
    }

    public double getAngleDegrees() {
        return getAngle() * 360;
    }
}
