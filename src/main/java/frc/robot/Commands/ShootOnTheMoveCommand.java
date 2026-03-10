package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.TurretSystem.FlywheelSubsystem;
import frc.robot.subsystems.TurretSystem.TurretSubsystem;

import java.util.function.Supplier;


public class ShootOnTheMoveCommand extends Command
{

  private final Supplier<Pose2d>        robotPose;
  private final Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds;
  private final Pose2d                  goalPose;

  private TurretSubsystem m_turret;
  // HoodSubsystem disabled - hood not in use
  private FlywheelSubsystem m_launcher;

  // Tuned Constants
  /**
   * Time in seconds between when the robot is told to move and when the shooter actually shoots.
   */
  private final double                     latency      = 0.3; 
  /**
   * Flywheel diameter in meters (4 inches)
   */
  private static final double              FLYWHEEL_DIAMETER_METERS = 0.1016;
  /**
   * Maps Distance (meters) to exit velocity (m/s)
   */
  private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

  /** NetworkTable used to expose shooter table entries for live tuning. */
  private final NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("SOTM/ShooterTable");

  /**
   * Default distance/velocity pairs. Values are pushed to NetworkTables on
   * construction so they appear in SmartDashboard/Elastic immediately.
   * Edit them live on the dashboard — the table is rebuilt every execute() loop.
   */
  private static final double[] DEFAULT_DISTANCES  = { 1.0,  1.5,  2.0,  2.5,  3.0,  3.5,  4.0,  4.5,  5.0,  5.5 };
  private static final double[] DEFAULT_VELOCITIES = { 0.0,  0.0, 15.0, 16.0, 16.5, 17.3, 18.6, 21.3, 23.9, 29.2 };


  public ShootOnTheMoveCommand(Supplier<Pose2d> currentPose, Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
                               Pose2d goal, TurretSubsystem turret, FlywheelSubsystem launcher)
  {
    m_launcher = launcher; //flywheel
    m_turret = turret;




    robotPose = currentPose;
    this.fieldOrientedChassisSpeeds = fieldOrientedChassisSpeeds;
    this.goalPose = goal;

    // Build the interpolation table from hardcoded defaults on first construction.
    // Call rebuildFromDashboard() (bind it to a button) to reload from NT/dashboard.
    for (int i = 0; i < DEFAULT_DISTANCES.length; i++) {
      shooterTable.put(DEFAULT_DISTANCES[i], DEFAULT_VELOCITIES[i]);
    }

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

  /**
   * Returns a Command that publishes the current table to NetworkTables (if not
   * already there) then reads back whatever values are on the dashboard and
   * reloads the interpolation table.  Bind this to a controller button so you
   * can tweak values in Elastic/SmartDashboard and apply them without redeploying.
   *
   * <p>Usage in RobotContainer:
   * <pre>
   *   controller.a().onTrue(shootOnTheMove.rebuildFromDashboard());
   * </pre>
   */
  public Command rebuildFromDashboard() {
    return Commands.runOnce(() -> {
      shooterTable.clear();
      for (int i = 0; i < DEFAULT_DISTANCES.length; i++) {
        String key      = DEFAULT_DISTANCES[i] + "m (m/s)";
        // setDefaultDouble only writes if the key doesn't already exist,
        // so hand-edited dashboard values are always preserved.
        tuningTable.getEntry(key).setDefaultDouble(DEFAULT_VELOCITIES[i]);
        double velocity = tuningTable.getEntry(key).getDouble(DEFAULT_VELOCITIES[i]);
        shooterTable.put(DEFAULT_DISTANCES[i], velocity);
      }
      SmartDashboard.putBoolean("SOTM: Table Reloaded", true);
    });
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

    // Debug: show how much the velocity compensation shifted the turret angle
    double uncompensatedAngle = targetVec.getAngle().getDegrees();
    SmartDashboard.putNumber("SOTM: Uncompensated Field Angle", uncompensatedAngle);
    SmartDashboard.putNumber("SOTM: Compensated Field Angle", fieldSpaceTurretAngle);
    SmartDashboard.putNumber("SOTM: Angle Correction (deg)", fieldSpaceTurretAngle - uncompensatedAngle);

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

    // 6. CONVERT COMPENSATED VELOCITY TO RPM
    // newHorizontalSpeed is the compensated exit speed (m/s) after vector subtraction
    double requiredRPM = calculateRPMFromVelocity(newHorizontalSpeed);

    // Clamp turret angle to soft limits
    double clampedTurretAngle = MathUtil.clamp(turretAngle, TurretSubsystem.softLimitMin.in(Degrees), TurretSubsystem.softLimitMax.in(Degrees));

    SmartDashboard.putNumber("SOTM: Clamped Turret Angle", clampedTurretAngle);
    SmartDashboard.putNumber("SOTM: Required RPM", requiredRPM);

    // 7. SET OUTPUTS
    m_turret.setAngleSetpoint(Degrees.of(clampedTurretAngle)); // Could also just set the swerveDrive to point towards this angle like AlignToGoal
    // m_hood.setAngle() - hood disabled
    m_launcher.setVelocitySetpoint(RPM.of(requiredRPM));
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