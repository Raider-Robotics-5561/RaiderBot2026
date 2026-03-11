package frc.robot.util;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.robot.Setpoints.Indexer;
// import frc.robot.Setpoints.Turret.Flywheel;
import frc.robot.Commands.AlignToGoal;
import frc.robot.Commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.HopperSysytem.HopperExtenderSubsystem;
import frc.robot.subsystems.HopperSysytem.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.HopperSysytem.HopperRollerSubsystem;
import frc.robot.subsystems.HopperSysytem.KickerSubsystem;
import frc.robot.util.TurretSystem.FlywheelSubsystem;
import frc.robot.util.TurretSystem.HoodSubsystem;
import frc.robot.util.TurretSystem.TurretSubsystem;
// import frc.robot.systems.ShooterTargetingSystem.Shot;
import yams.mechanisms.velocity.FlyWheel;

public class ScoringSystem {

  private KickerSubsystem m_kicker;
  private HopperExtenderSubsystem m_hopperExtender;
  private IntakeSubsystem m_intakeRollers;
  private SwerveSubsystem m_swerve;
  private TurretSubsystem m_turret;
  private HoodSubsystem m_hood;
  private FlywheelSubsystem m_flywheel;
  private HopperRollerSubsystem m_bellyrollers;
  private ShooterTargetingSystem m_shooterAimer; 
  private AlignToGoal aim;
  private ShootOnTheMoveCommand SOTM;

  public ScoringSystem(
      KickerSubsystem indexer,
      HopperRollerSubsystem bellyrollers,
      HopperExtenderSubsystem intakeArm,
      IntakeSubsystem intakeRoller,
      SwerveSubsystem swerve,
      TurretSubsystem turret, FlywheelSubsystem turretFlywheel, HoodSubsystem hood, KickerSubsystem kicker, ShootOnTheMoveCommand sotm) {
    m_kicker = indexer;
    m_bellyrollers = bellyrollers;
    m_hopperExtender = intakeArm;
    m_intakeRollers = intakeRoller;
    m_swerve = swerve;
    m_turret = turret;
    m_hood = hood;
    m_kicker = kicker;
    m_flywheel = turretFlywheel;
    SOTM = sotm;
  }
  

  public Command aim(){
    return null;
  }
  

  public Command score() {
    // this one included turret tracking
    return SOTM;
  }

  public Command shootBall() {

    // just transfer and shoot
   // return m_turret.setAngle(Degrees.zero()).alongWith(m_flywheel.setVelocity(RPM.of(3500)), m_bellyrollers.setDutyCycle(0.6), m_kicker.indexShoot());
    return Commands.none();
  }
}