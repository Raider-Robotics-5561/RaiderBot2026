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
//NOTE - Check below
//CONTINUE WORKING ON THE ENTIRE SOTM METHOD FOLLOWING THIS REPO: https://github.com/BroncBotz3481/FRC2026/blob/main/src/main/java/frc/robot/systems/ShooterTargetingSystem.java
//Currently "copying" their method but will work to fit our needs. 12am code time isnt the time for this. 
//Tomorrow will be the time to make this work for us as well as adding 2 controllers.



/**
 * Largely written by Eeshwar based off their blog at https://blog.eeshwark.com/robotblog/shooting-on-the-fly
 */
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
  private final double                     latency      = 0.15;
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
                             Pair.of(Meters.of(4), RPM.of(3500)))
    )
    {shooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));}

    addRequirements();
  }

  @Override
  public void initialize()
  {

  }

  @Override
  public void execute()
  {
    // Please look here for the original authors work!
    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // YASS did not come up with this
    // -------------------------------------------------------

    var robotSpeed = fieldOrientedChassisSpeeds.get();
    // 1. LATENCY COMP
    Translation2d futurePos = robotPose.get().getTranslation().plus(
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
                                                                   );

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = goalPose.getTranslation();
    Translation2d targetVec    = goalLocation.minus(futurePos);
    double        dist         = targetVec.getNorm();

    // 3. CALCULATE IDEAL SHOT (Stationary)
    // Note: This returns HORIZONTAL velocity component
    double idealHorizontalSpeed = shooterTable.get(dist);

    // 4. VECTOR SUBTRACTION
    Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    Translation2d shotVec     = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

    // 5. CONVERT TO CONTROLS
    double turretAngle        = shotVec.getAngle().getDegrees();
    double newHorizontalSpeed = shotVec.getNorm();

    // 6. SOLVE FOR NEW PITCH/RPM
    // Assuming constant total exit velocity, variable hood:
    double totalExitVelocity = 15.0; // m/s
    // Clamp to avoid domain errors if we need more speed than possible
    double ratio    = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
    double newPitch = Math.acos(ratio);
    double clampedPitch = MathUtil.clamp(newPitch, HoodSubsystem.softLimitMin.in(Rotations), HoodSubsystem.softLimitMax.in(Rotations));
    double clampedTurretAngle = -MathUtil.clamp(turretAngle, TurretSubsystem.softLimitMin.in(Degrees), TurretSubsystem.softLimitMax.in(Degrees));
    //Drive team can move robot 


    SmartDashboard.putNumber("SOTM: Clamped Turret Angle", clampedTurretAngle);
    SmartDashboard.putNumber("SOTM: Clamped Hood Angle", clampedPitch);
    SmartDashboard.putNumber("SOTM: Total Exit Velocity", totalExitVelocity);

    // 7. SET OUTPUTS
    m_turret.setAngleSetpoint(Degrees.of(clampedTurretAngle)); // Could also just set the swerveDrive to point towards this angle like AlignToGoal
    // m_hood.setAngle(Degrees.of(Math.toDegrees(clampedPitch)));
    // m_launcher.setRPM(MetersPerSecond.of(totalExitVelocity));
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

  }
}