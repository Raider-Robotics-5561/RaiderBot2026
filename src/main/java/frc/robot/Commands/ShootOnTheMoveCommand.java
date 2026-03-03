package frc.robot.Commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSystem.FlywheelSubsystem;
import frc.robot.subsystems.TurretSystem.HoodSubsystem;
import frc.robot.subsystems.TurretSystem.ShooterSubsystem;
import frc.robot.subsystems.TurretSystem.TurretSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

import java.util.function.Supplier;
import yams.mechanisms.swerve.SwerveDrive;


/**
 * Adapted from 6328 Mechanical Advantage!
 * Original source is here: https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java
 */
public class ShootOnTheMoveCommand extends Command
{

  private final double     loopPeriodSecs = Milliseconds.of(20).in(Seconds);
  // Outputs
  private       Rotation2d lastTurretAngle;
  private       double     lastHoodAngle;
  private       Rotation2d turretAngle;
  private       double     hoodAngle      = Double.NaN;

  // Private Variables
  private              TurretSubsystem                          turret;
  private              FlywheelSubsystem                        flywheelSubsystem;
  private              HoodSubsystem                            hoodSubsystem;
  private              Supplier<ChassisSpeeds>                  _fieldRelativeVelocity;
  private              Supplier<Pose2d>                         estimatedPose;
  private              Field2d                                  debugField             = new Field2d();
  private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap     =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap               launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap               timeOfFlightMap        =
      new InterpolatingDoubleTreeMap();

  // Tuning Constants
  private final Debouncer shootingDebounce = new Debouncer(0.1, DebounceType.kFalling);
  private final double    phaseDelay       = 0.05;
  private       Distance  minDistance      = Feet.of(1);
  private       Distance  maxDistance      = Meters.of(5);

  static
  {
    //NOTE - NEED OUR OWN VALUES IN THESE MAPS, THESE ARE JUST EXAMPLES
    // These should be found on your robot
    //Postition on the field / distance from hub plus our value from 
    launchHoodAngleMap.put(1.0, Rotation2d.fromRotations(0));
    launchHoodAngleMap.put(1.5, Rotation2d.fromRotations(0));
    launchHoodAngleMap.put(2.0, Rotation2d.fromRotations(0));
    launchHoodAngleMap.put(2.5, Rotation2d.fromRotations(0));
    launchHoodAngleMap.put(3.0, Rotation2d.fromRotations(0));
    launchHoodAngleMap.put(3.5, Rotation2d.fromRotations(0));
    launchHoodAngleMap.put(4.0, Rotation2d.fromRotations(0));
    launchHoodAngleMap.put(4.5, Rotation2d.fromRotations(0));
    launchHoodAngleMap.put(5.0, Rotation2d.fromRotations(0));
    launchHoodAngleMap.put(5.5, Rotation2d.fromRotations(0));

    launchFlywheelSpeedMap.put(1.0, 0.0);
    launchFlywheelSpeedMap.put(1.5, 0.0);
    launchFlywheelSpeedMap.put(2.0, 3500.0);
    launchFlywheelSpeedMap.put(2.5, 0.0);
    launchFlywheelSpeedMap.put(3.0, 0.0);
    launchFlywheelSpeedMap.put(3.5, 0.0);
    launchFlywheelSpeedMap.put(4.0, 0.0);
    launchFlywheelSpeedMap.put(4.5, 0.0);
    launchFlywheelSpeedMap.put(5.0, 0.0);
    launchFlywheelSpeedMap.put(5.5, 0.0);

    timeOfFlightMap.put(1.0, 0.0);
    timeOfFlightMap.put(1.5, 0.0);
    timeOfFlightMap.put(2.0, 1.11); //Note - TOF was not recorded past 1.5m
    timeOfFlightMap.put(2.5, 1.09);
    timeOfFlightMap.put(3.0, 0.90);
  }

  public ShootOnTheMoveCommand(TurretSubsystem turret, ShooterSubsystem shooter, HoodSubsystem hood,
                               SwerveDrive swerveDrive)
  {
    SmartDashboard.putData("ShootOnTheMoveField", debugField);
    estimatedPose = () -> {
      // Calculate estimated pose while accounting for phase delay
      ChassisSpeeds robotRelativeVelocity = swerveDrive.getRobotRelativeSpeed();
      var           robotPose             = swerveDrive.getPose();
      robotPose = robotPose.exp(
          new Twist2d(
              robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
              robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
              robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));
      // Optional, add logging here
      debugField.setRobotPose(robotPose);
      return robotPose;
    };
    _fieldRelativeVelocity = swerveDrive::getFieldRelativeSpeed;

    this.turret = turret;

    addRequirements(turret, flywheelSubsystem, hoodSubsystem);}
    
    
    
      @Override
  public void initialize()
  {

  }

  @Override
  public void execute()
  {
    // Get estimated pose
    var robotPose             = estimatedPose.get();
    var fieldRelativeVelocity = _fieldRelativeVelocity.get();

    // Calculate distance from turret to target
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d turretPosition         = turret.getPose(robotPose);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());
    
    // Calculate field relative turret velocity
    Angle         robotAngle     = robotPose.getRotation().getMeasure();
    ChassisSpeeds turretVelocity = turret.getVelocity(fieldRelativeVelocity, robotAngle);

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose                   = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++)
    {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocity.vxMetersPerSecond * timeOfFlight;
      double offsetY = turretVelocity.vyMetersPerSecond * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    hoodAngle = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
    if (lastTurretAngle == null) {lastTurretAngle = turretAngle;}
    if (Double.isNaN(lastHoodAngle)) {lastHoodAngle = hoodAngle;}
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    var lookaheadTurretToTargetDistanceMeasure = Meters.of(lookaheadTurretToTargetDistance);
    if (lookaheadTurretToTargetDistanceMeasure.gte(minDistance) &&
        lookaheadTurretToTargetDistanceMeasure.lte(maxDistance))
    {
      var shooterRPM = RPM.of(launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));
      turret.setAngleSetpoint(turretAngle.getMeasure());
      hoodSubsystem.setAngleSetpoint(Radians.of(hoodAngle));
      flywheelSubsystem.setVelocitySetpoint(shooterRPM);
      if (shootingDebounce.calculate(flywheelSubsystem.getVelocity().isNear(shooterRPM, RPM.of(10))))
      {
        // Set indexer active
        // HERE
      }
    }

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
    flywheelSubsystem.setDutyCycleSetpoint(0);
  }
}