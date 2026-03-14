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
  private final Supplier<Pose2d>        goalPose;

  private TurretSubsystem m_turret;
  // HoodSubsystem disabled - hood not in use
  private FlywheelSubsystem m_launcher;

  /**
   * When true the command never self-terminates (teleop toggle use).
   * When false the command exits once both subsystems reach their setpoints (auto use).
   */
  private final boolean runContinuously;

  /**
   * Called each loop while turret and flywheel are both at setpoint — starts the kicker/belly feed.
   * Called in end() to ensure feed always stops when the command ends.
   */
  private final Runnable feedOn;
  private final Runnable feedOff;

  // Tuned Constants
  /**
   * When true, the turret-tip tangential velocity from robot rotation (e.g. ShakeCommand omega)
   * is included in the SOTM shot vector calculation.
   * Set to false to revert to the old behaviour (translational velocity only).
   */
  private static final boolean COMPENSATE_FOR_ROTATION = true;

  /**
   * Time in seconds between when the robot is told to move and when the shooter actually shoots.
   */
  private final double                     latency      = 0.4; 
  /**
   * Flywheel diameter in meters (4 inches)
   */
  private static final double              FLYWHEEL_DIAMETER_METERS = 0.1016;
  /**
   * Maps Distance (meters) to flywheel RPM
   */
  private final static InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

  /** NetworkTable used to expose shooter table entries for live tuning. */
  private final static NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("SOTM/ShooterTable");

  /**
   * Default distance/RPM pairs. Values are pushed to NetworkTables on
   * construction so they appear in SmartDashboard/Elastic immediately.
   * Edit them live on the dashboard — the table is rebuilt every execute() loop.
   */
  private static final double[] DEFAULT_DISTANCES  = { 1.0,   1.5,   2.0,   2.5,   3.0,   3.5,   4.0,   4.5,   5.0,   5.5 };
  private static final double[] DEFAULT_VELOCITIES = { 0.0, 0.0, 2800.0, 2900.0, 3050.0, 3150.0, 3200.0, 3350.0, 3500.0, 3650.0 };

  /** Static initializer to populate NetworkTable only once */
  static {
    for (int i = 0; i < DEFAULT_DISTANCES.length; i++) {
      String key = DEFAULT_DISTANCES[i] + "m (RPM)";
      tuningTable.getEntry(key).setDefaultDouble(DEFAULT_VELOCITIES[i]);
      shooterTable.put(DEFAULT_DISTANCES[i], DEFAULT_VELOCITIES[i]);
    }
  }



  /**
   * Teleop constructor — runs continuously until cancelled by the button toggle.
   * Accepts a fixed {@code Pose2d} goal. No feed control.
   */
  public ShootOnTheMoveCommand(Supplier<Pose2d> currentPose, Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
                               Pose2d goal, TurretSubsystem turret, FlywheelSubsystem launcher)
  {
    this(currentPose, fieldOrientedChassisSpeeds, () -> goal, turret, launcher, true, () -> {}, () -> {});
  }

  /**
   * Auto constructor — exits once both subsystems reach their setpoints.
   * Accepts a fixed {@code Pose2d} goal. No feed control.
   */
  public ShootOnTheMoveCommand(Supplier<Pose2d> currentPose, Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
                               Pose2d goal, TurretSubsystem turret, FlywheelSubsystem launcher,
                               boolean runContinuously)
  {
    this(currentPose, fieldOrientedChassisSpeeds, () -> goal, turret, launcher, runContinuously, () -> {}, () -> {});
  }

  /**
   * Full teleop constructor with feed control.
   * Goal is a {@code Supplier<Pose2d>} evaluated each loop.
   *
   * @param feedOn  called each loop when turret + flywheel are both at setpoint
   * @param feedOff called when the command ends to stop the feed
   */
  public ShootOnTheMoveCommand(Supplier<Pose2d> currentPose, Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
                               Supplier<Pose2d> goal, TurretSubsystem turret, FlywheelSubsystem launcher,
                               boolean runContinuously, Runnable feedOn, Runnable feedOff)
  {
    this.runContinuously = runContinuously;
    this.feedOn  = feedOn;
    this.feedOff = feedOff;
    m_launcher = launcher;
    m_turret = turret;
    robotPose = currentPose;
    this.fieldOrientedChassisSpeeds = fieldOrientedChassisSpeeds;
    this.goalPose = goal;
    addRequirements(turret, launcher);
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
  public static Command rebuildFromDashboard() {
    return Commands.runOnce(() -> {
      shooterTable.clear();
      for (int i = 0; i < DEFAULT_DISTANCES.length; i++) {
        String key      = DEFAULT_DISTANCES[i] + "m (RPM)";
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

    // 1. Calculate Time of Flight (ToF) FIRST based on current distance
    double currentDist = robotPose.get().getTranslation().getDistance(goalPose.get().getTranslation());
    double staticRPM = shooterTable.get(currentDist);
    double staticVel = calculateVelocityFromRPM(staticRPM);
    double ballFlightTime = currentDist / staticVel;

    // 2. NOW calculate Total Latency
    double totalLatency = latency + ballFlightTime; 

    // 1. LATENCY COMP
    Translation2d futurePos = robotPose.get().getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(totalLatency));

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = goalPose.get().getTranslation();
    Translation2d targetVec    = goalLocation.minus(futurePos);
    double        dist         = targetVec.getNorm();
    SmartDashboard.putNumber("SOTM: dist", dist);


    // 3. CALCULATE IDEAL SHOT (Stationary)
    // Note: shooterTable stores RPM, but we need m/s for vector math
    double idealHorizontalSpeedRPM = shooterTable.get(dist);
    double flywheelRadiusMeters = FLYWHEEL_DIAMETER_METERS / 2;
    double idealHorizontalSpeedMs = (idealHorizontalSpeedRPM * 2 * Math.PI * flywheelRadiusMeters) / 60;

    SmartDashboard.putNumber("SOTM: Ideal Horizontal Speed (RPM)", idealHorizontalSpeedRPM);

    // 4. VECTOR SUBTRACTION
    // Use turret-tip velocity instead of raw robot center velocity.
    // This accounts for any angular velocity (e.g. from ShakeCommand) that causes
    // the turret — which is offset from the robot center — to have an additional
    // tangential linear velocity that would otherwise throw off the shot vector.
    // Toggle COMPENSATE_FOR_ROTATION to false to revert to translational-only compensation.
    ChassisSpeeds turretTipSpeeds = COMPENSATE_FOR_ROTATION
        ? m_turret.getVelocity(robotSpeed, robotPose.get().getRotation().getMeasure())
        : robotSpeed;
    Translation2d robotVelVec = new Translation2d(turretTipSpeeds.vxMetersPerSecond,
                                                  turretTipSpeeds.vyMetersPerSecond);
    Translation2d shotVec     = targetVec.div(dist).times(idealHorizontalSpeedMs).plus(robotVelVec); //.times(1)); Why was this here?

    // 5. CONVERT TO CONTROLS
    double fieldSpaceTurretAngle = shotVec.getAngle().getDegrees();
    double newHorizontalSpeed    = shotVec.getNorm();
    

    
    // Debug: show how much the velocity compensation shifted the turret angle
    double uncompensatedAngle = targetVec.getAngle().getDegrees();
    SmartDashboard.putNumber("SOTM: Uncompensated Field Angle", uncompensatedAngle);
    SmartDashboard.putNumber("SOTM: Compensated Field Angle", fieldSpaceTurretAngle);
    SmartDashboard.putNumber("SOTM: Angle Correction (deg)", fieldSpaceTurretAngle - uncompensatedAngle);
    // Show shake-induced tangential velocity so it's visible during tuning
    SmartDashboard.putNumber("SOTM: Shake Omega (rad-s)", robotSpeed.omegaRadiansPerSecond);
    SmartDashboard.putNumber("SOTM: Turret Tip vX (m-s)", turretTipSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("SOTM: Turret Tip vY (m-s)", turretTipSpeeds.vyMetersPerSecond);

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

    // 6. CONVERT TO RPM
    // The shooterTable.get(dist) already gives us RPM, but we need to account for
    // the velocity compensation from vector subtraction.
    // Since newHorizontalSpeed is still calculated in m/s, we use the conversion function
    double requiredRPM = -calculateRPMFromVelocity(newHorizontalSpeed);

    // Clamp turret angle to soft limits
    double clampedTurretAngle = -MathUtil.clamp(turretAngle, TurretSubsystem.softLimitMin.in(Degrees), TurretSubsystem.softLimitMax.in(Degrees));
    // double angleDifference = Math.abs(clampedTurretAngle - m_turret.getAngle().in(Degrees));

    SmartDashboard.putNumber("SOTM: Clamped Turret Angle", clampedTurretAngle);
    SmartDashboard.putNumber("SOTM: Required RPM", requiredRPM);

    // 7. SET OUTPUTS
    m_turret.setAngleSetpoint(Degrees.of(clampedTurretAngle));
    // m_hood.setAngle() - hood disabled
    m_launcher.setVelocitySetpoint(RPM.of(requiredRPM));

    // 8. FEED CONTROL — engage kicker/belly as soon as both subsystems are on target
    SmartDashboard.putBoolean("SOTM: Ready to Fire", m_launcher.atSetpoint() && m_turret.atSetpoint());
    if (m_launcher.atSetpoint() && m_turret.atSetpoint()) {
      feedOn.run();
    } else {
      feedOff.run();
    }
  }

    /** Helper to reverse your RPM calculation */
    private double calculateVelocityFromRPM(double rpm) { 
       double flywheelRadiusMeters = FLYWHEEL_DIAMETER_METERS / 2;
       return (rpm * 2 * Math.PI * flywheelRadiusMeters) / 60.0;
    }   

  @Override
  public boolean isFinished()
  {
    if (runContinuously) return false;
    return m_launcher.atSetpoint() && m_turret.atSetpoint();
  }

  @Override
  public void end(boolean interrupted)
  {
    // Stop feed, flywheel, and turret when command ends
    feedOff.run();
    m_launcher.setVelocitySetpoint(RPM.of(0));
    m_turret.setAngleSetpoint(Degrees.of(0));
  }
}