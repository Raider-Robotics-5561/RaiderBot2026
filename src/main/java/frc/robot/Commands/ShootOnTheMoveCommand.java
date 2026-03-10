package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSystem.HoodSubsystem;
import frc.robot.subsystems.TurretSystem.FlywheelSubsystem;
import frc.robot.subsystems.TurretSystem.TurretSubsystem;

import java.util.List;
import java.util.function.Supplier;


public class ShootOnTheMoveCommand extends Command
{

  private final Supplier<Pose2d>        robotPose;
  private final Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds;
  private final Pose2d                  goalPose;

  private TurretSubsystem m_turret;
  private HoodSubsystem m_hood;
  private FlywheelSubsystem m_launcher;

  // Tuned Constants
  /**
   * Time in seconds between when the robot is told to move and when the shooter actually shoots.
   */
  private final double                     latency      = 0.001;
  /**
   * Flywheel diameter in meters (4 inches)
   */
  private static final double              FLYWHEEL_DIAMETER_METERS = 0.1016;
  /**
   * Maps Distance to RPM
   */
  private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();


  public ShootOnTheMoveCommand(Supplier<Pose2d> currentPose, Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
                               Pose2d goal, TurretSubsystem turret, HoodSubsystem hood, FlywheelSubsystem launcher)
  {
    m_launcher = launcher; //flywheel
    m_turret = turret;
    m_hood = hood;




    robotPose = currentPose;
    this.fieldOrientedChassisSpeeds = fieldOrientedChassisSpeeds;
    this.goalPose = goal;

    // Test Results
    //MAKE MORE POINTS FOR DISTANCE, FLYWHEEL SPEED, HOOD ANGLE
    for (var entry : List.of(Pair.of(Meters.of(1), RPM.of((0))),
                             Pair.of(Meters.of(1.5), RPM.of(0)),
                             Pair.of(Meters.of(2.0), RPM.of(2800)),
                             Pair.of(Meters.of(2.5), RPM.of(3000)),
                             Pair.of(Meters.of(3), RPM.of(3100)),
                             Pair.of(Meters.of(3.5), RPM.of(3250)),
                             Pair.of(Meters.of(4), RPM.of(3500)),
                             Pair.of(Meters.of(4.5), RPM.of(4000)),
                             Pair.of(Meters.of(5), RPM.of(4500)),
                             Pair.of(Meters.of(5.5), RPM.of(5500)))
    )
    {shooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));}

    addRequirements();
  }

  /**
   * Converts linear exit velocity (m/s) to flywheel RPM
   * @param exitVelocity the desired exit velocity in m/s
   * @return the required flywheel RPM
   */
  private double calculateRPMFromVelocity(double exitVelocity) {
    double flywheelRadiusMeters = FLYWHEEL_DIAMETER_METERS / 2;
    return (exitVelocity * 60) / (2 * Math.PI * flywheelRadiusMeters);
  }

  @Override
  public void initialize()
  {

  }

  @Override
  public void execute()
  {
    var robotSpeed = fieldOrientedChassisSpeeds.get();
    // 1. LATENCY COMP
    Translation2d futurePos = robotPose.get().getTranslation().plus(
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
                                                                   );

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = goalPose.getTranslation();
    Translation2d targetVec    = goalLocation.minus(futurePos);
    double        dist         = targetVec.getNorm();

    SmartDashboard.putNumber("SOTM: dist", dist);

    // 3. CALCULATE IDEAL SHOT (Stationary)
    // Note: This returns HORIZONTAL velocity component
    double idealHorizontalSpeed = shooterTable.get(dist);

    SmartDashboard.putNumber("SOTM: Ideal Horizontal Speed", idealHorizontalSpeed);

    // 4. VECTOR SUBTRACTION
    Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    Translation2d shotVec     = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

    // 5. CONVERT TO CONTROLS
    double fieldSpaceTurretAngle = shotVec.getAngle().getDegrees();
    double newHorizontalSpeed    = shotVec.getNorm();

    // 5b. ACCOUNT FOR ROBOT ROTATION (Gyro compensation)
    // Convert from field-space to robot-space by subtracting the robot's heading
    double robotHeadingDegrees = robotPose.get().getRotation().getDegrees();
    SmartDashboard.putNumber("SOTM: Robot Heading", robotHeadingDegrees);

    // Add 180 degrees because turret is mounted facing backwards
    double turretAngle = fieldSpaceTurretAngle - robotHeadingDegrees - 180;
    // Normalize angle to [-180, 180] range using proper modulo arithmetic
    while (turretAngle > 180) {
      turretAngle -= 360;
    }
    while (turretAngle < -180) {
      turretAngle += 360;
    }

    // 6. SOLVE FOR NEW PITCH/RPM
    // Get RPM from shooter table based on distance
    double requiredRPM = -shooterTable.get(dist);
    
    // Convert RPM back to exit velocity to calculate pitch
    double totalExitVelocity = (requiredRPM * 2 * Math.PI * (FLYWHEEL_DIAMETER_METERS / 2)) / 60;
    // Clamp to avoid domain errors if we need more speed than possible
    double ratio    = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
    double newPitch = Math.acos(ratio);
    double clampedPitch = MathUtil.clamp(newPitch, HoodSubsystem.softLimitMin.in(Rotations), HoodSubsystem.softLimitMax.in(Rotations));
    // Convert turret angle to rotations to match soft limit units, then clamp
    double turretAngleRotations = turretAngle / 360.0;
    double clampedTurretAngleRotations = MathUtil.clamp(turretAngleRotations, TurretSubsystem.softLimitMin.in(Rotations), TurretSubsystem.softLimitMax.in(Rotations));
    double clampedTurretAngle = -clampedTurretAngleRotations * 360.0;
    //Drive team can move robot 


    SmartDashboard.putNumber("SOTM: Clamped Turret Angle", clampedTurretAngle);
    SmartDashboard.putNumber("SOTM: Clamped Hood Angle", clampedPitch);
    SmartDashboard.putNumber("SOTM: Required RPM", requiredRPM);

    // 7. SET OUTPUTS
    m_turret.setAngleSetpoint(Degrees.of(clampedTurretAngle)); // Could also just set the swerveDrive to point towards this angle like AlignToGoal
    m_hood.setAngle(Degrees.of(Math.toDegrees(clampedPitch)));
    m_launcher.setVelocitySetpoint(RPM.of(requiredRPM));
    // NOTE - Disabled for testing
  }

  @Override
  public boolean isFinished()
  {
    // TODO: Make this return true when this Command no longer needs to run execute()

    
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    // Stop all subsystems when command ends
    m_launcher.setVelocitySetpoint(RPM.of(0));
    m_turret.setAngleSetpoint(Degrees.of(0));
  }
}