package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ShakeCommand extends Command {

    // Total shake swing: ±3 degrees (6 degrees total)
    private static final double AMPLITUDE_RAD = Units.degreesToRadians(4.0);
    // One full oscillation every 0.5 seconds
    private static final double PERIOD_SECONDS = 0.5;

    private final SwerveSubsystem m_swerve;
    private final Timer m_timer = new Timer();

    public ShakeCommand(SwerveSubsystem swerve) {
        m_swerve = swerve;
        addRequirements();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    // Drives a sinusoidal rotation: omega = A * (2π/T) * cos(2π * t / T)
    // This is the derivative of: angle(t) = A * sin(2π * t / T)
    // so the robot rocks ±3° continuously at a 0.5s period.
    @Override
    public void execute() {
        double t = m_timer.get();
        double omega = AMPLITUDE_RAD * (2 * Math.PI / PERIOD_SECONDS)
                       * Math.cos(2 * Math.PI * t / PERIOD_SECONDS);
        m_swerve.drive(new Translation2d(), omega, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(new Translation2d(), 0, false);
    }

    // Runs until interrupted (e.g. button released)
    @Override
    public boolean isFinished() {
        return false;
    }
}
