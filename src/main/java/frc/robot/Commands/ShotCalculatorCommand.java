package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSysytem.BellyRollerSubsystem;
import frc.robot.subsystems.HopperSysytem.KickerSubsystem;
import frc.robot.subsystems.TurretSystem.FlywheelSubsystem;
import frc.robot.subsystems.TurretSystem.TurretSubsystem;
import frc.robot.util.ShotCalculator;

import java.util.function.Supplier;

/**
 * Shoot-on-the-move command backed by {@link ShotCalculator}.
 *
 * <p>
 * The {@code ShotCalculator} handles all the math: Newton-method time-of-flight
 * solving,
 * drag compensation, latency-compensated pose prediction, and a 0-100
 * confidence score.
 * This command just feeds inputs in, reads the result out, and drives the
 * hardware.
 *
 * <p>
 * <b>LUT data</b> (from the hand-tuned ShootOnTheMoveCommand table):
 * 
 * <pre>
 *   dist (m) | RPM    | ToF (s)
 *   ---------+--------+--------
 *   1.0      |    0.0 | 0.88
 *   1.5      |    0.0 | 0.92
 *   2.0      | 2800.0 | 0.96
 *   2.5      | 2900.0 | 1.00
 *   3.0      | 3050.0 | 1.04
 *   3.5      | 3150.0 | 1.08
 *   4.0      | 3200.0 | 1.11
 *   4.5      | 3350.0 | 1.15
 *   5.0      | 3500.0 | 1.19
 *   5.5      | 3650.0 | 1.23
 * </pre>
 *
 * <p>
 * <b>Teleop use</b> — toggle on/off with a button; runs continuously:
 * 
 * <pre>
 * ShotCalculatorCommand sotm = new ShotCalculatorCommand(
 *     swerve::getPose,
 *     swerve::getFieldVelocity,
 *     swerve::getRobotVelocity,
 *     goalPose,
 *     turret, flywheel,
 *     kicker, bellyRoller);
 *
 * driverController.rightBumper().toggleOnTrue(sotm);
 * </pre>
 *
 * <p>
 * <b>Auto use</b> — self-terminates after a timeout:
 * 
 * <pre>
 *   new ShotCalculatorCommand(..., 3.0 /* seconds &#42;/);
 * </pre>
 */
public class ShotCalculatorCommand extends Command {

  // ── LUT data pulled from ShootOnTheMoveCommand ──────────────────────────────
  private static final double[] LUT_DISTANCES = { 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5 };
  private static final double[] LUT_RPMS =      { 0.0, 0.0, 2750.0, 2800.0, 3000.0, 3150.0, 3200.0, 3450.0, 3550.0, 3650.0, 3800.0, 3950.0 }; //NOTE - Lets tune these.
  private static final double[] LUT_TOFS =      { 0.88, 0.92, 0.96, 1.00, 1.04, 1.08, 1.11, 1.15, 1.19, 1.23, 1.27, 1.31 };

  // ── Confidence threshold — below this we don't feed the ball ────────────────
  /**
   * Minimum confidence score (0-100) required to release the note into the
   * launcher.
   */
  private static final double FIRE_CONFIDENCE_THRESHOLD = 55.0;

  // ── Subsystem references ────────────────────────────────────────────────────
  private final TurretSubsystem m_turret;
  private final FlywheelSubsystem m_flywheel;
  private final KickerSubsystem m_kicker; // nullable
  private final BellyRollerSubsystem m_hopperRoller; // nullable

  // ── Input suppliers ─────────────────────────────────────────────────────────
  private final Supplier<Pose2d> m_robotPose;
  private final Supplier<ChassisSpeeds> m_fieldVelocity;
  private final Supplier<ChassisSpeeds> m_robotVelocity;
  private final Supplier<Pose2d> m_goalPose;

  // ── Core solver ─────────────────────────────────────────────────────────────
  private final ShotCalculator m_calc;

  // ── Auto timeout ────────────────────────────────────────────────────────────
  /**
   * Seconds before the command self-terminates. {@code <= 0} means run
   * continuously
   * (teleop toggle use).
   */
  private final double m_timeoutSeconds;
  private final Timer m_timer = new Timer();

  // ── Vision confidence passthrough ────────────────────────────────────────────
  /**
   * Supplier for a 0-1 vision confidence value. Pass {@code () -> 1.0} if you
   * have no vision system, or wire it to your PhotonVision pipeline confidence.
   */
  private final Supplier<Double> m_visionConfidence;

  // ─────────────────────────────────────────────────────────────────────────────
  // Constructors
  // ─────────────────────────────────────────────────────────────────────────────

  /**
   * Full constructor. Prefer the factory-style constructors below for common
   * cases.
   *
   * @param robotPose        current field-relative robot pose
   * @param fieldVelocity    field-relative chassis speeds (from swerve odometry)
   * @param robotVelocity    robot-relative chassis speeds (from swerve
   *                         kinematics)
   * @param goal             supplier for the goal {@link Pose2d} (evaluated each
   *                         loop)
   * @param turret           turret subsystem
   * @param flywheel         flywheel subsystem
   * @param timeoutSeconds   auto timeout; use {@code 0} for continuous teleop use
   * @param kicker           kicker subsystem; may be {@code null}
   * @param hopperRoller     belly roller subsystem; may be {@code null}
   * @param visionConfidence supplier returning 0-1 pipeline confidence; use
   *                         {@code () -> 1.0}
   *                         if no vision
   */
  public ShotCalculatorCommand(
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> fieldVelocity,
      Supplier<ChassisSpeeds> robotVelocity,
      Supplier<Pose2d> goal,
      TurretSubsystem turret,
      FlywheelSubsystem flywheel,
      double timeoutSeconds,
      KickerSubsystem kicker,
      BellyRollerSubsystem hopperRoller,
      Supplier<Double> visionConfidence) {

    m_robotPose = robotPose;
    m_fieldVelocity = fieldVelocity;
    m_robotVelocity = robotVelocity;
    m_goalPose = goal;
    m_turret = turret;
    m_flywheel = flywheel;
    m_timeoutSeconds = timeoutSeconds;
    m_kicker = kicker;
    m_hopperRoller = hopperRoller;
    m_visionConfidence = visionConfidence;

    // Build and configure the calculator
    ShotCalculator.Config cfg = new ShotCalculator.Config();
    // Turret is mounted 1.5 ft behind robot center (negative X) — match
    // roboToTurret offset
    cfg.launcherOffsetX = -0.06471; // -1.5 ft in meters
    cfg.launcherOffsetY = -0.00305; // meters left of robot center
    cfg.minScoringDistance = 1.0;
    cfg.maxScoringDistance = 5.5;
    // 30 ms vision latency + 20 ms mechanical latency (consistent with
    // ShootOnTheMoveCommand's 0.3 s total)
    cfg.phaseDelayMs = 30.0;
    cfg.mechLatencyMs = 20.0;
    // Drag coefficient carried over from the SOTM solver
    cfg.sotmDragCoeff = 0.47;
    m_calc = new ShotCalculator(cfg);

    // Load the LUT entries
    for (int i = 0; i < LUT_DISTANCES.length; i++) {
      m_calc.loadLUTEntry(LUT_DISTANCES[i], LUT_RPMS[i], LUT_TOFS[i]);
    }

    // Declare subsystem requirements
    if (kicker != null && hopperRoller != null) {
      addRequirements(turret, flywheel, kicker, hopperRoller);
    } else {
      addRequirements(turret, flywheel);
    }
  }

  // ── Convenience constructors ─────────────────────────────────────────────────

  /**
   * Teleop constructor — no feed control, runs continuously until cancelled.
   * Uses a fixed goal pose.
   */
  public ShotCalculatorCommand(
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> fieldVelocity,
      Supplier<ChassisSpeeds> robotVelocity,
      Supplier<Pose2d> goal,
      TurretSubsystem turret,
      FlywheelSubsystem flywheel) {
    this(robotPose, fieldVelocity, robotVelocity,
        goal, turret, flywheel,
        0, null, null, () -> 1.0);
  }

  /**
   * Teleop constructor with feed control — runs continuously until cancelled.
   * Uses a fixed goal pose.
   */
  public ShotCalculatorCommand(
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> fieldVelocity,
      Supplier<ChassisSpeeds> robotVelocity,
      Supplier<Pose2d> goal,
      TurretSubsystem turret,
      FlywheelSubsystem flywheel,
      KickerSubsystem kicker,
      BellyRollerSubsystem hopperRoller) {
    this(robotPose, fieldVelocity, robotVelocity,
        goal, turret, flywheel,
        0, kicker, hopperRoller, () -> 1.0);
  }

  /**
   * Auto constructor — self-terminates after {@code timeoutSeconds}.
   * Uses a fixed goal pose and no feed control.
   */
  public ShotCalculatorCommand(
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> fieldVelocity,
      Supplier<ChassisSpeeds> robotVelocity,
      Pose2d goal,
      TurretSubsystem turret,
      FlywheelSubsystem flywheel,
      double timeoutSeconds) {
    this(robotPose, fieldVelocity, robotVelocity,
        () -> goal, turret, flywheel,
        timeoutSeconds, null, null, () -> 1.0);
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Command lifecycle
  // ─────────────────────────────────────────────────────────────────────────────

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_calc.resetWarmStart();
  }

  @Override
  public void execute() {
    // System.out.println("Executing ShotCalculatorCommand...");

    Pose2d pose = m_robotPose.get();
    ChassisSpeeds fieldVel = m_fieldVelocity.get();
    ChassisSpeeds robotVel = m_robotVelocity.get();
    Translation2d hubCenter = m_goalPose.get().getTranslation();

    System.out.println("Hub pose: (" + hubCenter.getX() + ", " + hubCenter.getY() + ")");

    // Hub forward vector: points from the hub toward the robot so the
    // behind-hub gate works correctly.
    Translation2d toRobot = pose.getTranslation().minus(hubCenter);
    Translation2d hubForward = toRobot.getNorm() > 0.01
        ? toRobot.div(toRobot.getNorm())
        : new Translation2d(1, 0);

    double visionConf = 1; //Setting this to one

    ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
        pose, fieldVel, robotVel, hubCenter, hubForward, visionConf);

    ShotCalculator.LaunchParameters result = m_calc.calculate(inputs);

    if (!result.isValid()) {
      System.out.println("ShotCalculator returned invalid result!");
    }

    // ── Telemetry ──────────────────────────────────────────────────────────────
    SmartDashboard.putBoolean("SOTM2: Valid", result.isValid());
    SmartDashboard.putNumber("SOTM2: Confidence", result.confidence());
    SmartDashboard.putNumber("SOTM2: RPM", result.rpm());
    SmartDashboard.putNumber("SOTM2: ToF (s)", result.timeOfFlightSec());
    SmartDashboard.putNumber("SOTM2: Drive Angle", result.driveAngle().getDegrees());
    SmartDashboard.putNumber("SOTM2: Solved Dist (m)", result.solvedDistanceM());
    SmartDashboard.putNumber("SOTM2: Iterations", result.iterationsUsed());
    SmartDashboard.putBoolean("SOTM2: Warm Start", result.warmStartUsed());

    if (!result.isValid()) {
      // Out of range or bad inputs — hold turret at zero, spin down
      m_turret.setAngleSetpoint(Degrees.of(0));
      m_flywheel.setVelocitySetpoint(RPM.of(0));
      stopFeed();
      SmartDashboard.putBoolean("SOTM2: Ready to Fire", false);
      return;
    }

    // ── Drive angle → robot-relative turret angle ──────────────────────────────
    // The turret is mounted facing backwards, so add 180° before converting.
    double fieldSpaceTurretAngleDeg = result.driveAngle().getDegrees();
    double robotHeadingDeg = pose.getRotation().getDegrees();
    double turretRelAngle = fieldSpaceTurretAngleDeg - robotHeadingDeg + 180.0;

    // Normalize to (-180, 180)
    turretRelAngle = MathUtil.angleModulus(Math.toRadians(turretRelAngle));
    turretRelAngle = Math.toDegrees(turretRelAngle);


    double minDeg = TurretSubsystem.softLimitMin.in(Degrees);
    double maxDeg = TurretSubsystem.softLimitMax.in(Degrees);
    double clampedAngle = -MathUtil.clamp(turretRelAngle, minDeg, maxDeg);

    SmartDashboard.putNumber("SOTM2: Turret Angle (clamped)", clampedAngle);

    // ── Apply setpoints ────────────────────────────────────────────────────────
    // RPM from the solver is already negative by convention (launcher fires
    // backwards)
    m_turret.setAngleSetpoint(Degrees.of(clampedAngle));
    m_flywheel.setVelocitySetpoint(RPM.of(-result.rpm()));

    // ── Feed control ──────────────────────────────────────────────────────────
    boolean readyToFire = m_flywheel.atSetpoint()
        && m_turret.atSetpoint()
        && result.confidence() >= FIRE_CONFIDENCE_THRESHOLD;

    System.out.println("FIRE_CONFIDENCE_THRESHOLD:" + FIRE_CONFIDENCE_THRESHOLD);
    SmartDashboard.putBoolean("SOTM2: Ready to Fire", readyToFire);

    if (m_kicker != null && m_hopperRoller != null) {
      if (readyToFire) {
        m_kicker.setDutyCycleDirect(-0.9);
        m_hopperRoller.setVelocityDirect(RPM.of(-2500));
      } else {
        // Keep kicker primed but hold the ball until we're on target
        m_kicker.setDutyCycleDirect(0); // REVIEW - Not sure if we need to keep this on
        m_hopperRoller.setDutyCycleDirect(0);
      }
    }
  }

  @Override
  public boolean isFinished() {
    if (m_timeoutSeconds <= 0)
      return false;
    return m_timer.hasElapsed(m_timeoutSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    stopFeed();
    m_flywheel.setVelocitySetpoint(RPM.of(0));
    m_turret.setAngleSetpoint(Degrees.of(0));
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Helpers
  // ─────────────────────────────────────────────────────────────────────────────

  private void stopFeed() {
    if (m_kicker != null && m_hopperRoller != null) {
      m_kicker.setDutyCycleDirect(0);
      m_hopperRoller.setDutyCycleDirect(0);
    }
  }

  /**
   * Expose the underlying {@link ShotCalculator} so you can call
   * {@code adjustOffset()}, {@code addRpmCorrection()}, etc. from RobotContainer.
   */
  public ShotCalculator getCalculator() {
    return m_calc;
  }
}
