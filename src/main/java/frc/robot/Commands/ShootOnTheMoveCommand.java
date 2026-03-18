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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.HopperSysytem.BellyRollerSubsystem;
import frc.robot.subsystems.HopperSysytem.KickerSubsystem;
import frc.robot.subsystems.TurretSystem.FlywheelSubsystem;
import frc.robot.subsystems.TurretSystem.TurretSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;


public class ShootOnTheMoveCommand extends Command
{

  private final Supplier<Pose2d>        robotPose;
  private final Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds;
  private final Supplier<Pose2d>        goalPose;

  private TurretSubsystem m_turret;
  // HoodSubsystem disabled - hood not in use
  private FlywheelSubsystem m_launcher;

  private static final InterpolatingDoubleTreeMap               timeOfFlightMap        =
      new InterpolatingDoubleTreeMap();

      static {    
      //NOTE - MUST PUT CORRECT VALUES
      timeOfFlightMap.put(5.5, 1.23);
      timeOfFlightMap.put(5.0, 1.19);
      timeOfFlightMap.put(4.5, 1.15);
      timeOfFlightMap.put(4.0, 1.11);
      timeOfFlightMap.put(3.5, 1.08);
      timeOfFlightMap.put(3.0, 1.04);
      timeOfFlightMap.put(2.5, 1.00);
      timeOfFlightMap.put(2.0, 0.96);
      timeOfFlightMap.put(1.5, 0.92);
      timeOfFlightMap.put(1.0, 0.88);
      }
  /**
   * Kicker and hopper roller subsystems used for feeding.
   * When non-null, SOTM owns these subsystems and drives them directly
   * instead of scheduling external commands (avoids subsystem conflicts).
   */
  private final KickerSubsystem m_kicker;
  private final BellyRollerSubsystem m_hopperRoller;

  /**
   * Timeout in seconds for auto use. When positive, the command will self-terminate
   * after this duration even if setpoints are not reached. A value of {@code 0}
   * (or negative) means the command runs continuously until externally cancelled
   * (teleop toggle use).
   */
  private final double timeoutSeconds;

  /** Timer used to enforce the auto timeout. */
  private final Timer m_timer = new Timer();

  // Tuned Constants
  /**
   * When non-null, rotation compensation is only active while this supplier returns true
   * (e.g. pass {@code shakeCommand::isScheduled} so compensation only runs during a shake).
   * When null, the static {@code COMPENSATE_FOR_ROTATION} flag governs the behaviour.
   */
  private final BooleanSupplier shakeActive;

  /**
   * Master enable for turret-tip rotation compensation.
   * Set to false to disable completely regardless of {@code shakeActive}.
   */
  private static final boolean COMPENSATE_FOR_ROTATION = true;

  /**
   * Time in seconds between when the robot is told to move and when the shooter actually shoots.
   */
  private final double                     latency      = 0.3; 
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
    this(currentPose, fieldOrientedChassisSpeeds, () -> goal, turret, launcher, 0, null, null, () -> true);
  }

  /**
   * Auto constructor — exits after {@code timeoutSeconds} or once both subsystems reach
   * their setpoints (whichever comes first). Pass {@code 0} to run continuously.
   * Accepts a fixed {@code Pose2d} goal. No feed control.
   */
  public ShootOnTheMoveCommand(Supplier<Pose2d> currentPose, Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
                               Pose2d goal, TurretSubsystem turret, FlywheelSubsystem launcher,
                               double timeoutSeconds)
  {
    this(currentPose, fieldOrientedChassisSpeeds, () -> goal, turret, launcher, timeoutSeconds, null, null, () -> true);
  }

  /**
   * Full constructor with feed control.
   * Goal is a {@code Supplier<Pose2d>} evaluated each loop.
   * SOTM directly requires and controls the kicker and hopper roller subsystems
   * to avoid subsystem conflicts with external bindings.
   *
   * @param timeoutSeconds seconds before the command self-terminates; use {@code 0}
   *                       (or negative) to run continuously (teleop toggle use)
   * @param kicker         kicker subsystem (may be null if no feed control needed)
   * @param hopperRoller   hopper roller subsystem (may be null if no feed control needed)
   * @param shakeActive    returns true while ShakeCommand is running; rotation compensation
   *                       is only applied when this returns true (pass {@code shakeCmd::isScheduled})
   */
  public ShootOnTheMoveCommand(Supplier<Pose2d> currentPose, Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
                               Supplier<Pose2d> goal, TurretSubsystem turret, FlywheelSubsystem launcher,
                               double timeoutSeconds, KickerSubsystem kicker, BellyRollerSubsystem hopperRoller,
                               BooleanSupplier shakeActive)
  {
    this.timeoutSeconds = timeoutSeconds;
    this.m_kicker = kicker;
    this.m_hopperRoller = hopperRoller;
    m_launcher = launcher;
    m_turret = turret;
    robotPose = currentPose;
    this.fieldOrientedChassisSpeeds = fieldOrientedChassisSpeeds;
    this.goalPose = goal;
    this.shakeActive = shakeActive;
    // Require turret + flywheel always; also require kicker + hopper if provided
    // so the scheduler properly interrupts any conflicting commands on those subsystems.
    if (kicker != null && hopperRoller != null) {
      addRequirements(turret, launcher, kicker, hopperRoller);
    } else {
      addRequirements(turret, launcher);
    }
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
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute()
  {
    var robotSpeed = fieldOrientedChassisSpeeds.get();

    // 1. Calculate Time of Flight (ToF) FIRST based on current distance
    double currentDist = robotPose.get().getTranslation().getDistance(goalPose.get().getTranslation());
    double staticRPM = shooterTable.get(currentDist);
    double staticVel = calculateVelocityFromRPM(staticRPM);
    double ballFlightTime = staticVel > 1e-6 ? currentDist / staticVel : 0.0;

    // 2. NOW calculate Total Latency (initial guess)
    double totalLatency = latency + ballFlightTime;

    // 1. LATENCY COMP (initial estimate)
    Translation2d futurePos = robotPose.get().getTranslation().plus(
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(totalLatency)); // May need to change this to just the latency

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = goalPose.get().getTranslation();
    Translation2d targetVec    = goalLocation.minus(futurePos);
    double        dist         = targetVec.getNorm();
    SmartDashboard.putNumber("SOTM: dist", dist);

    // 3. CALCULATE IDEAL SHOT (Stationary)
    // shooterTable stores RPM, use helper to convert to m/s
    double idealHorizontalSpeedRPM = shooterTable.get(dist);
    double idealHorizontalSpeedMs = calculateVelocityFromRPM(idealHorizontalSpeedRPM);

    SmartDashboard.putNumber("SOTM: Ideal Horizontal Speed (RPM)", idealHorizontalSpeedRPM);

    // 4. VECTOR SUBTRACTION
    boolean doRotationComp = COMPENSATE_FOR_ROTATION && (shakeActive == null ? true : shakeActive.getAsBoolean());
    ChassisSpeeds turretTipSpeeds = doRotationComp
        ? m_turret.getVelocity(robotSpeed, robotPose.get().getRotation().getMeasure())
        : robotSpeed;
    Translation2d robotVelVec = new Translation2d(turretTipSpeeds.vxMetersPerSecond,
                                                  turretTipSpeeds.vyMetersPerSecond);
    // shot vector: direction to target * ideal stationary speed, plus turret-tip linear velocity
    Translation2d shotVec     = dist > 0 ? targetVec.div(dist).times(idealHorizontalSpeedMs).plus(robotVelVec)
                                        : robotVelVec;

    // 5. CONVERT TO CONTROLS (field-space turret angle + required horizontal speed)
    double fieldSpaceTurretAngle = shotVec.getAngle().getDegrees();
    double newHorizontalSpeed    = shotVec.getNorm();

    // Debug info
    double uncompensatedAngle = targetVec.getAngle().getDegrees();
    SmartDashboard.putNumber("SOTM: Uncompensated Field Angle", uncompensatedAngle);
    SmartDashboard.putNumber("SOTM: Compensated Field Angle", fieldSpaceTurretAngle);
    SmartDashboard.putNumber("SOTM: Angle Correction (deg)", fieldSpaceTurretAngle - uncompensatedAngle);
    SmartDashboard.putBoolean("SOTM: Shake Compensation Active", doRotationComp);
    SmartDashboard.putNumber("SOTM: Shake Omega (rad-s)", robotSpeed.omegaRadiansPerSecond);
    SmartDashboard.putNumber("SOTM: Turret Tip vX (m-s)", turretTipSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("SOTM: Turret Tip vY (m-s)", turretTipSpeeds.vyMetersPerSecond);

    // 6. CONVERT TO RPM (note: keep sign convention used by your flywheel subsystem)
    double requiredRPM = -calculateRPMFromVelocity(newHorizontalSpeed);
    SmartDashboard.putNumber("SOTM: Pre-TOF Required RPM", requiredRPM);

    // --- Real Time-of-Flight (ToF) correction (iterative) ---
    double totalLatencyIter = latency;
    final int maxIters = 8;
    final double tol = 1e-3;
    Translation2d futurePosIter = robotPose.get().getTranslation();
    double distIter = currentDist;
    for (int i = 0; i < maxIters; i++) {
      // project robot forward by current latency estimate
      futurePosIter = robotPose.get().getTranslation().plus(
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(totalLatencyIter));
      // distance at the (projected) shot moment
      distIter = goalPose.get().getTranslation().minus(futurePosIter).getNorm();

      // get ToF either from tuning map or approximate from shooter table
      double tofFromMap = timeOfFlightMap.get(distIter);
      double tof;
      if (Double.isFinite(tofFromMap) && tofFromMap > 0.0) {
        tof = tofFromMap;
      } else {
        double rpmForDist = shooterTable.get(distIter);
        double velForDist = calculateVelocityFromRPM(rpmForDist);
        tof = velForDist > 1e-6 ? distIter / velForDist : 0.0;
      }

      double newTotal = latency + tof;
      if (Math.abs(newTotal - totalLatencyIter) < tol) {
        totalLatencyIter = newTotal;
        break;
      }
      totalLatencyIter = newTotal;
    }

    // Recompute using converged latency
    futurePos = robotPose.get().getTranslation().plus(
      new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(totalLatencyIter)); // Changed to standard latency not full latency + ToF
    targetVec = goalPose.get().getTranslation().minus(futurePos);
    dist = targetVec.getNorm();

    // ideal horizontal speed for corrected distance
    idealHorizontalSpeedRPM = shooterTable.get(dist);
    idealHorizontalSpeedMs = calculateVelocityFromRPM(idealHorizontalSpeedRPM);

    // recompute turret-tip compensation
    turretTipSpeeds = doRotationComp
      ? m_turret.getVelocity(robotSpeed, robotPose.get().getRotation().getMeasure())
      : robotSpeed;
    robotVelVec = new Translation2d(turretTipSpeeds.vxMetersPerSecond, turretTipSpeeds.vyMetersPerSecond);
    shotVec = dist > 0 ? targetVec.div(dist).times(idealHorizontalSpeedMs).plus(robotVelVec)
                       : robotVelVec;

    fieldSpaceTurretAngle = shotVec.getAngle().getDegrees();
    newHorizontalSpeed = shotVec.getNorm();
    requiredRPM = -calculateRPMFromVelocity(newHorizontalSpeed);

    // Convert field-space turret angle to turret-relative and clamp to soft limits
    double robotHeadingDegrees = robotPose.get().getRotation().getDegrees();

    // Add 180 degrees because turret is mounted facing backwards.
    // Convert field angle -> robot-relative turret angle.
    double correctedTurretAngle = fieldSpaceTurretAngle - robotHeadingDegrees + 180.0;

    // Normalize to (-180, 180)
    while (correctedTurretAngle > 180) correctedTurretAngle -= 360;
    while (correctedTurretAngle <= -180) correctedTurretAngle += 360;

    // Clamp to allowed turret travel. TurretSubsystem.softLimitMin/Max should represent -90..90.
    double minLimitDeg = TurretSubsystem.softLimitMin.in(Degrees);
    double maxLimitDeg = TurretSubsystem.softLimitMax.in(Degrees);
    double clampedTurretAngle = -MathUtil.clamp(correctedTurretAngle, minLimitDeg, maxLimitDeg);

    SmartDashboard.putNumber("SOTM: Corrected Turret Angle (pre-clamp)", correctedTurretAngle);
    SmartDashboard.putNumber("SOTM: Clamped Turret Angle", clampedTurretAngle);
    SmartDashboard.putNumber("SOTM: Required RPM", requiredRPM);

    // 7. SET OUTPUTS
    m_turret.setAngleSetpoint(Degrees.of(clampedTurretAngle));
    m_launcher.setVelocitySetpoint(RPM.of(requiredRPM));

    // 8. FEED CONTROL — engage kicker/belly as soon as both subsystems are on target
    SmartDashboard.putBoolean("SOTM: Ready to Fire", m_launcher.atSetpoint() && m_turret.atSetpoint());
    if (m_kicker != null && m_hopperRoller != null) {
      if (m_launcher.atSetpoint() && m_turret.atSetpoint()) {
        m_kicker.setDutyCycleDirect(-0.9);
        m_hopperRoller.setVelocityDirect(RPM.of(2500));
      } else {
        m_kicker.setDutyCycleDirect(-0.9);
        m_hopperRoller.setDutyCycleDirect(0);
      }
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
    // Continuous mode (teleop) — never self-terminate
    if (timeoutSeconds <= 0) return false;
    // Auto mode — finish only when the timer expires
    return m_timer.hasElapsed(timeoutSeconds);
  }

  @Override
  public void end(boolean interrupted)
  {
    m_timer.stop();
    // Stop feed, flywheel, and turret when command ends
    if (m_kicker != null && m_hopperRoller != null) {
      m_kicker.setDutyCycleDirect(0);
      m_hopperRoller.setDutyCycleDirect(0);
    }
    m_launcher.setVelocitySetpoint(RPM.of(0));
    m_turret.setAngleSetpoint(Degrees.of(0));
  }
}