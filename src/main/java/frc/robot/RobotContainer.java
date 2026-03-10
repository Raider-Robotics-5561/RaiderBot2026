package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.ClimberDownCommand;
import frc.robot.Commands.ClimberUpCommand;
import frc.robot.Commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SuperStructure;
import swervelib.SwerveInputStream;

public class RobotContainer {
	// Subsystem inizialization (Should only be Climber and superstructure)
	private final SuperStructure SuperStructure = new SuperStructure();
	public final ClimberSubsystem m_climber = new ClimberSubsystem();

	// Controller initialization
	final CommandXboxController DriveController = new CommandXboxController(0);
	final CommandXboxController OperatorController = new CommandXboxController(1);

	// NOTE - This is for Dashboard control - True = Dashbaord setting enabled,
	// False = no dashboard control
	boolean ManualControl = true;

	// Swerve initialization
	public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

	// Goal Position for SOTM
	private Pose2d goalPos = new Pose2d(Inches.of(182.11), Inches.of(158.84), new Rotation3d(0, 0, 0).toRotation2d()); // Note
																														// -
																														// Update
																														// ME\
	// Drive to pose testing poition (Red alliance)
	private Pose2d DriveToPose = new Pose2d(Meters.of(14), Meters.of(1.5), new Rotation3d(0, 0, 0).toRotation2d()); // Note
																													// -
																													// Update
																													// ME

	// ======================Auton_Config=========================
	private final Command Teston;
	private final Command APP1;
	private final Command APP2;
	private final Command Nuetral_Left;
	private final Command Nuetral_Right;
	private final Command A_Center_1;
	SendableChooser<Command> m_chooser;

	// =======================================================

	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
			() -> DriveController.getLeftY() * -1,
			() -> DriveController.getLeftX() * -1)
			// .withControllerRotationAxis(() -> DriveController.getRawAxis(2))
			.withControllerRotationAxis(() -> DriveController.getRightX() * -1)
			.deadband(Constants.MiscConstants.DEADBAND)
			.scaleTranslation(0.20)
			.scaleRotation(0.35)
			.allianceRelativeControl(true);

	// Clone's the angular velocity input stream and converts it to a
	// fieldRelative input stream.
	SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
			.withControllerHeadingAxis(DriveController::getLeftX, DriveController::getLeftY)
			.headingWhile(true);

	// Clone's the angular velocity input stream and converts it to a
	// robotRelative input stream.
	SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(false)
			.allianceRelativeControl(true);

	SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
			() -> -DriveController.getLeftY(),
			() -> -DriveController.getLeftX())
			.withControllerRotationAxis(() -> DriveController.getRawAxis(2))
			.deadband(Constants.MiscConstants.DEADBAND)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);

	// Derive the heading axis with math!
	SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
			.withControllerHeadingAxis(() -> Math.sin(DriveController.getRawAxis(2) *
					Math.PI) * (Math.PI * 2),
					() -> Math.cos(DriveController.getRawAxis(2) *
							Math.PI) * (Math.PI * 2))
			.headingWhile(true);

	// The container for the robot. Contains subsystems, OI devices, and commands.

	public RobotContainer() {
		NamedCommands.registerCommand("ShootOnTheMoveCommand", 
  				new ShootOnTheMoveCommand(drivebase::getPose,
										  drivebase::getRobotVelocity,
										  goalPos,
										  SuperStructure.TurretSubsytem,
										  SuperStructure.HoodSubsystem,
										  SuperStructure.FlywheelSubsystem));
										

		NamedCommands.registerCommand("DeployHopper", SuperStructure.SetHopperPos());
				
		
		NamedCommands.registerCommand("RetractHopper", Commands.run(() -> {
  				SuperStructure.SetHopperPosZero();
				}));
		
		NamedCommands.registerCommand("IntakeRollerOn", SuperStructure.SetIntakePWR(-0.8));
		NamedCommands.registerCommand("BellyFeed", SuperStructure.SetKickerAndBelly());

  				

		NamedCommands.registerCommand("IntakeRollerOff", Commands.run(() -> {
  				SuperStructure.SetIntakePWR(0);
				}));	
				
		// Secondary Auton Configs
		Teston = drivebase.getAutonomousCommand("Teston");
		APP1 = drivebase.getAutonomousCommand("APP1");
		APP2 = drivebase.getAutonomousCommand("APP2");
		Nuetral_Left = drivebase.getAutonomousCommand("Nuetral_Left");
		Nuetral_Right = drivebase.getAutonomousCommand("Nuetral_Right");
		A_Center_1 = drivebase.getAutonomousCommand("A_Center_1");

		m_chooser = new SendableChooser<Command>();
		m_chooser.addOption("Teston", Teston);
		m_chooser.addOption("APP1", APP1);
		m_chooser.addOption("APP2", APP2);
		m_chooser.addOption("Nuetral_Left", Nuetral_Left);
		m_chooser.addOption("Nuetral_Right", Nuetral_Right);
		m_chooser.addOption("A_Center_1", A_Center_1);
		SmartDashboard.putData(m_chooser);

		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);

		}

	// Button Bindings for Drive and Operator controllers
	private void configureBindings() {

		Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
		drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
	
	// Operator Controls
	// Intake Rollers
	OperatorController.b().onTrue(SuperStructure.SetIntakePWR(-0.8)); // This should should be neative to run the intake
																		// in
	OperatorController.y().onTrue(SuperStructure.SetIntakePWR(0));

	// Hood Homeing
	OperatorController.povLeft().onTrue(SuperStructure.HoodSubsystem.homing());

	// Kicker and Belly Control
	OperatorController.a().onTrue(SuperStructure.SetKickerAndBelly());
	OperatorController.x().onTrue(SuperStructure.SetKickerBellyOff());
	OperatorController.povUp().onTrue(SuperStructure.BackDriveKicker());

	// Hopper Extender Control
	OperatorController.leftBumper().onTrue(SuperStructure.SetHopperExtenderPower(0.3))
	.or(OperatorController.rightBumper().onTrue(SuperStructure.SetHopperExtenderPower(-0.3)))
	.whileFalse(SuperStructure.SetHopperExtenderPower(0));

	/* ~~~~~~~~~~~~~~~~~~Drive Control~~~~~~~~~~~~~~~~~~~~~~~~ */
	if(ManualControl==true){
		// Manual Dashboard control for hood, flywheel, and turret
		DriveController.y().onTrue(Commands.runOnce(() -> {
			double hoodReq = SmartDashboard.getNumber("Hood Angle Requested", 0);
			double turretReq = SmartDashboard.getNumber("Turret Angle Requested", 0);
			double flyReq = SmartDashboard.getNumber("Flywheel RPM Requested", 0);
			System.out.println("A pressed: hood=" + hoodReq + " turret=" + turretReq + " fly=" + flyReq);
			SuperStructure.HoodSubsystem.setAngleSetpoint(Degrees.of(hoodReq));
			SuperStructure.TurretSubsytem.setAngleSetpoint(Degrees.of(turretReq));
			SuperStructure.FlywheelSubsystem.setVelocitySetpoint(RPM.of(flyReq));
		}, SuperStructure.HoodSubsystem, SuperStructure.TurretSubsytem, SuperStructure.FlywheelSubsystem));
	}

	DriveController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
	DriveController.povUp().onTrue(SuperStructure.SetHopperPos());
	DriveController.povDown().onTrue(SuperStructure.SetHopperPosZero());

	// DriveController.x().whileTrue(drivebase.driveToPose(DriveToPose));
	DriveController.leftBumper().whileTrue(new ClimberUpCommand(m_climber));
	DriveController.rightBumper().whileTrue(new ClimberDownCommand(m_climber));
	DriveController.x().toggleOnTrue(new ShootOnTheMoveCommand(drivebase::getPose,
															drivebase::getRobotVelocity,
															goalPos,
															SuperStructure.TurretSubsytem,
															SuperStructure.HoodSubsystem,
															SuperStructure.FlywheelSubsystem));
						
															


	// This is our boost control Right Trigger
	DriveController.axisGreaterThan(3,0.01).onChange(Commands.runOnce(()->
	{
		driveAngularVelocity.scaleTranslation(DriveController.getRightTriggerAxis() +
				0.35);
		driveAngularVelocity.scaleRotation((DriveController.getRightTriggerAxis() *
				Constants.MiscConstants.RotationSpeedScale) + 0.25);
	}).repeatedly()).whileFalse(Commands.runOnce(()->
	{
		driveAngularVelocity.scaleTranslation(0.25);
		driveAngularVelocity.scaleRotation(0.15);
	}).repeatedly());
	}

	// Gets the selected autonomous command.
	public Command getAutonomousCommand() {
		return m_chooser.getSelected();
	}

	public void setMotorBrake(boolean brake)
	{
	drivebase.setMotorBrake(false);
	}
}