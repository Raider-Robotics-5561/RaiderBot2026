package frc.robot.util;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import yams.units.YUnits;

import static edu.wpi.first.units.Units.*;

public class Setpoints {

  public static class Climber {
      public static Distance startHeight = Meters.of(0);
      public static Distance lowHeight = Meters.of(0.6);
      public static Distance highHeight = Meters.of(0.9);

  }

  public static class Intake {

    public static Angle intakeArmStartAngle = Degrees.of(45);
    public static Angle intakeArmAngleIn = Degrees.of(10);
    public static Angle intakeArmAngleOut = Degrees.of(50);

  }

  public static class Indexer {
    // indexSubsystem and agitator
    public static final AngularVelocity startFlywheelAngle = YUnits.RPY.of(0);
  }

  public static class Turret {

    public static class Hood {
      public static final Angle startHoodAngle = Degrees.of(30);
      public static final Angle lowerHoodAngle = Degrees.of(20);
      public static final Angle higherHoodAngle = Degrees.of(45);
    }

    public static class Pivot {
      // these are random, check before running on real robot to avoid breaking
      public static final Angle startTurretAngle = Degree.of(0);
      public static final Angle leftTurretLimit = Degree.of(-135);
      public static final Angle rightTurretLimit = Degree.of(135);
    }

  }

  public static class AutoDriving {

    /* how to create offset
    *         public static final Transform2d offset = new Transform2d(Inches.of(24).in(Meters),
                                                                  Inches.of(0).in(Meters),
                                                                  Rotation2d.fromDegrees(0));
    */
    // x + front ->, y + left

    // create Classes here

  }
}