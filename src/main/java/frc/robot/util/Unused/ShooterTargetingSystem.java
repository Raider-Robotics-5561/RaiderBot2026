package frc.robot.util.Unused;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.TurretSystem.FlywheelSubsystem;
// import frc.robot.subsystems.TurretSystem.HoodSubsystem;
import frc.robot.subsystems.TurretSystem.TurretSubsystem;

public class ShooterTargetingSystem {

  double clearance;
  double velocity;
  double initialVelocity;
  double turretPitchAngle;
  public double turretAngle;
  double turretAngleRelative;
  double hubRadius;
  double hubHeight;
  double hubInsideHeight;
  Transform3d ROBOT_TO_TURRET;
  Transform2d ROBOT_TO_TURRET_2D;
  Pose2d goalLocation;
  Translation2d shotVec;
  public static Transform3d turretPivotCenterFromRobotCenter =
        new Transform3d(new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0)), new Rotation3d());
  public static Timer timer = new Timer();

  public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
    }

    // see https://www.desmos.com/geometry/l4edywkmha
    public static Angle calculateAngleFromVelocity(Pose2d robot, LinearVelocity velocity, Translation3d target) {
        double g = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);
        double vel = velocity.in(InchesPerSecond);
        double x_dist = getDistanceToTarget(robot, target).in(Inches);
        double y_dist = target.getMeasureZ()
                .minus(turretPivotCenterFromRobotCenter.getMeasureZ())
                .in(Inches);
        double angle = Math.atan(
                ((vel * vel) + Math.sqrt(Math.pow(vel, 4) - g * (g * x_dist * x_dist + 2 * y_dist * vel * vel)))
                        / (g * x_dist));
        return Radians.of(angle);
    }

    public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
    }

    public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
        return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
    }

    // calculates the angle of a turret relative to the robot to hit a target
    public static Angle calculateAzimuthAngle(Pose2d robot, Translation3d target, Angle currentAngle) {
        Translation2d turretTranslation = new Pose3d(robot)
                .transformBy(turretPivotCenterFromRobotCenter)
                .toPose2d()
                .getTranslation();

        Translation2d direction = target.toTranslation2d().minus(turretTranslation);
        return calculateAzimuthAngle(robot, direction.getAngle().getMeasure(), currentAngle);
    }

    // calculates the angle of a turret relative to the robot to hit a target
    public static Angle calculateAzimuthAngle(Pose2d robot, Angle fieldRelativeAngle, Angle currentAngle) {
        double angle = MathUtil.inputModulus(
                new Rotation2d(fieldRelativeAngle).minus(robot.getRotation()).getRotations(), -0.5, 0.5);
        double current = currentAngle.in(Rotations);
        if (current > 0 && angle + 1 <= TurretSubsystem.softLimitMin.in(Rotations)) angle += 1;
        if (current < 0 && angle - 1 >= TurretSubsystem.softLimitMax.in(Rotations)) angle -= 1;
        // Logger.recordOutput("Turret/DesiredAzimuthRad", angle);
        return Rotations.of(angle);
    }

  private final StructPublisher<Pose2d> shotVecPublisher =
    NetworkTableInstance.getDefault()
        .getTable("Drive")
        .getStructTopic("shotVecPublisher", Pose2d.struct)
        .publish();
  private final StructPublisher<Pose2d> newRobotPosePublisher =
    NetworkTableInstance.getDefault()
        .getTable("Drive")
        .getStructTopic("newRobotPosePublisher", Pose2d.struct)
        .publish();

  public ShooterTargetingSystem(Transform3d ROBOT_TO_TURRET) {

      clearance = Units.inchesToMeters(21); // clearance above goal (or smth idk)
      hubRadius = Units.inchesToMeters(24); // inches
      hubHeight = Units.inchesToMeters(72); // inches
      hubInsideHeight = Units.inchesToMeters(48); // inches
      this.ROBOT_TO_TURRET = ROBOT_TO_TURRET;
      ROBOT_TO_TURRET_2D =
          new Transform2d(this.ROBOT_TO_TURRET.getX(), this.ROBOT_TO_TURRET.getY(), new Rotation2d());

  }

   public static Shot findIdealVelocityAndAngle(Pose2d robotPose, ChassisSpeeds robotSpeed) {
    Transform3d ROBOT_TO_TURRET =
        new Transform3d(
            0, // back from robot center
            0, // centered left/right
            0.451739, // up from the floor reference
            new Rotation3d());
    Pose2d goalLocation =
        new Pose2d(
            Units.inchesToMeters(651.2 - 158.6 - 47.0 / 2),
            Units.inchesToMeters(317.7 / 2),
            new Rotation2d());
    double clearance = Units.inchesToMeters(21); // clearance above goal (or smth idk)
    double hubRadius = Units.inchesToMeters(24); // inches
    double hubHeight = Units.inchesToMeters(72); // inches
    double hubInsideHeight = Units.inchesToMeters(48); // inches
    double x1 =
        -1
            * new Pose3d(robotPose)
                .transformBy(ROBOT_TO_TURRET)
                .toPose2d()
                .getTranslation()
                .getDistance(goalLocation.getTranslation());
    // System.out.println("x1 = " + x1);
    double y1 = new Pose3d(robotPose).transformBy(ROBOT_TO_TURRET).getZ();
    double x2 = -1 * hubRadius;
    double y2 = hubHeight + clearance;
    double x3 = 0;
    double y3 = hubInsideHeight;
    // // System.out.println(
    //     "x1 = " + x1 + "\n" + "y1 = " + y1 + "\n" + "x2 = " + x2 + "\n" + "y2 = " + y2 + "\n"
    //         + "x3 = " + x3 + "\n" + "y3 = " + y3);
    double a =
        (y1 * (x2 - x3) + y2 * (-x1 + x3) + y3 * (x1 - x2)) / ((x1 - x2) * (x1 - x3) * (x2 - x3));
  
    double b =
        (-y1 * (x2 - x3) * (x2 + x3) + y2 * (x1 + x3) * (x1 - x3) - y3 * (x1 - x2) * (x1 + x2))
            / ((x1 - x2) * (x1 - x3) * (x2 - x3));


    Angle turretPitchAngle = Angle.ofBaseUnits(Math.atan(2 * a * x1 + b), Radians);

    LinearVelocity initialVelocity =
        LinearVelocity.ofBaseUnits(
            Math.sqrt(
                (-9.81) / (2 * a * Math.pow(Math.cos(turretPitchAngle.baseUnitMagnitude()), 2))),
            MetersPerSecond);

    // System.out.println(
    //     "a = "
    //         + a
    //         + "\n"
    //         + "b = "
    //         + b
    //         + "\n"
    //         + "turretPitchAngle = "
    //         + turretPitchAngle
    //         + "\n"
    //         + "initialVelocity = "
    //         + initialVelocity);

    return new Shot(initialVelocity, turretPitchAngle, Angle.ofBaseUnits(0, Radians));
  }


  public static Time calculateTimeOfFlight(
      LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
    double vel = exitVelocity.in(MetersPerSecond);
    double angle = hoodAngle.in(Radians);
    double dist = distance.in(Meters);
    return Seconds.of(dist / (vel * Math.cos(angle)));
  }

  public static Translation2d getTargetVector(Pose2d robotPose) {
    Pose2d goalLocation =
        new Pose2d(
            Units.inchesToMeters(651.2 - 158.6 - 47.0 / 2),
            Units.inchesToMeters(317.7 / 2),
            new Rotation2d());
    Translation2d targetVec = goalLocation.getTranslation().minus(robotPose.getTranslation());
    return targetVec;
    // double dist = targetVec.getNorm();
  }

  public static Pose2d getNewRobotPose(
      Pose2d robotPose, ChassisSpeeds chassisSpeeds, Time timeOfFlight) {
    Pose2d newRobotPose =
        robotPose.transformBy(
            new Transform2d(
                getRobotVelocityVector(chassisSpeeds).times(-1 * timeOfFlight.baseUnitMagnitude()),
                new Rotation2d()));
    return newRobotPose;
  }

  public static Translation2d getRobotVelocityVector(ChassisSpeeds chassisSpeeds) {
    Translation2d robotVelVec =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    return robotVelVec;
  }

  public static Pose2d getNewGoalLocation(
      Pose2d robotPose, ChassisSpeeds chassisSpeeds, Time timeOfFlight) {
    Pose2d goalLocation =
        new Pose2d(
            Units.inchesToMeters(651.2 - 158.6 - 47.0 / 2),
            Units.inchesToMeters(317.7 / 2),
            new Rotation2d());
    Pose2d newGoalLocation =
        goalLocation.transformBy(
            new Transform2d(
                getRobotVelocityVector(chassisSpeeds).times(-1 * timeOfFlight.baseUnitMagnitude()),
                new Rotation2d()));
    return newGoalLocation;
  }

  public static Translation2d getShotVector(Pose2d newGoalLocation, Pose2d robotPose) {
    Translation2d shotVec = newGoalLocation.getTranslation().minus(robotPose.getTranslation());
    return shotVec;
  }

  //   public double getVelocity() {
  //     return velocity;
  //   }

  //   public double getTurretPitchAngle() {
  //     return turretPitchAngle;
  //   }

  public static Shot getShotData(Pose2d oldRobotPose, ChassisSpeeds chassisSpeeds, int iterations) {
    Transform3d ROBOT_TO_TURRET =
        new Transform3d(
            0, // back from robot center
            0, // centered left/right
            0.451739, // up from the floor reference
            new Rotation3d());
    // 1. LATENCY COMP
    // System.out.println("shooterAimer time = " + timer.get());
    double latency = 0.37; // Tuned constant
    Pose2d robotPose =
        oldRobotPose
            // .transformBy(ROBOT_TO_TURRET.)
            .plus(
            new Transform2d(
                    chassisSpeeds.vxMetersPerSecond
                        * ((chassisSpeeds.vxMetersPerSecond < 0) ? -1 : 1),
                    chassisSpeeds.vyMetersPerSecond,
                    new Rotation2d())
                .times(-1 * latency));
    // System.out.println(
    //     "chassisSpeeds X thing = "
    //         + (chassisSpeeds.vxMetersPerSecond * ((chassisSpeeds.vxMetersPerSecond < 0) ? -1 : 1)));
    // System.out.println("chassisSpeeds Y thing = " + (chassisSpeeds.vyMetersPerSecond));

    Shot initialCalcShot = findIdealVelocityAndAngle(robotPose, chassisSpeeds);
    Translation2d targetVec = getTargetVector(robotPose);

    Time timeOfFlight =
        calculateTimeOfFlight(
            initialCalcShot.getVelocity(),
            initialCalcShot.getPitchAngle(),
            Distance.ofBaseUnits(targetVec.getNorm(), Meters));

    Pose2d newGoalLocation = getNewGoalLocation(robotPose, chassisSpeeds, timeOfFlight);
    Pose2d newRobotPose = getNewRobotPose(robotPose, chassisSpeeds, timeOfFlight);
    Translation2d shotVec = getShotVector(newGoalLocation, robotPose);

    for (int i = 0; i < iterations; i++) {
      newGoalLocation = getNewGoalLocation(robotPose, chassisSpeeds, timeOfFlight);
      newRobotPose = getNewRobotPose(robotPose, chassisSpeeds, timeOfFlight);

      targetVec = getTargetVector(robotPose);
      timeOfFlight =
          calculateTimeOfFlight(
              initialCalcShot.getVelocity(),
              initialCalcShot.getPitchAngle(),
              Distance.ofBaseUnits(targetVec.getNorm(), Meters));

      shotVec = getShotVector(newGoalLocation, robotPose);
    }

    Angle turretAngle = Angle.ofBaseUnits(shotVec.getAngle().getRadians(), Radians);
    Shot newCalcShot = findIdealVelocityAndAngle(newRobotPose, chassisSpeeds);
    Shot shot = new Shot(newCalcShot.getVelocity(), newCalcShot.getPitchAngle(), turretAngle);

    return shot;
  }

 


  public record Shot(double exitVelocity, double hoodAngle, double target) {
    public Shot(LinearVelocity exitVelocity, Angle hoodAngle, Angle target) {
      this(exitVelocity.in(MetersPerSecond), hoodAngle.in(Radians), target.in(Radians));
    }

    public LinearVelocity getVelocity() {
      return MetersPerSecond.of(this.exitVelocity);
    }

    public Angle getPitchAngle() {
      return Radians.of(this.hoodAngle);
    }

    public Angle getAngle() {
      return Radians.of(this.target);
    }
}
}
