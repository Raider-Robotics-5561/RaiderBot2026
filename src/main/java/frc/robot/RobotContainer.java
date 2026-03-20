package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static edu.wpi.first.units.Units.*;
import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.ShakeCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SuperStructure;
import swervelib.SwerveInputStream;
import frc.robot.Commands.ShotCalculatorCommand;

public class RobotContainer {
	// Subsystem inizialization (Should only be Climber and superstructure)
	private final SuperStructure SuperStructure = new SuperStructure();

	// Controller initialization
	final CommandXboxController DriveController = new CommandXboxController(0);
	final CommandXboxController OperatorController = new CommandXboxController(1);

	// NOTE - This is for Dashboard control 
	// True = Dashbaord setting enabled,
	// False = no dashboard control
	boolean ManualControl = false;

	// Swerve initialization
	public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

	// ======================Auton_Config=========================
	// Auto chooser stores names only; the command is built in getAutonomousCommand()
	// after gameInit() has registered all NamedCommands.
	SendableChooser<String> m_chooser;

	// =======================================================

	public Pose2d HubPose, AllianceWallDepot, AllianceWallOutpost;

	//Default to blue, but will be updated in gameInit() if the alliance is detected correctly
	public Alliance current_alliance = Alliance.Blue;

	// SOTM target — updated by D-pad buttons before the shared command is toggled.
	private Pose2d sotmTarget = null;

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
		//Rather than using a chooser of all of the complied autons, we can just use the string name to build the command in getAutonomousCommand() after gameInit() registers all of the NamedCommands.
		m_chooser = new SendableChooser<String>();
		m_chooser.addOption("Right_Trench_Double_ToOP", "Right_Trench_Double_ToOP");
		m_chooser.addOption("LeftNeutralToDP", "LeftNeutralToDP");
		m_chooser.addOption("SOTMtest", "SOTMtest");
		m_chooser.addOption("RightNuetralToOP", "RightNuetralToOP");
		m_chooser.addOption("APP1", "APP1");
		m_chooser.addOption("APP2", "APP2");
		SmartDashboard.putData(m_chooser);

		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);

	}

	//This runs when ever the bot is enabled, so it runs before auton and teleop. We use this to reset our sensors and set our default commands
	public void gameInit() {
		if(DriverStation.getAlliance().isPresent()) {
			current_alliance = DriverStation.getAlliance().get();

			if (current_alliance == DriverStation.Alliance.Blue) {
				HubPose = new Pose2d(Inches.of(182.11), Inches.of(158.84),
						new Rotation3d(0, 0, 0).toRotation2d());
				AllianceWallDepot = new Pose2d(Inches.of(12), Inches.of(24),
						new Rotation3d(0, 0, 0).toRotation2d());
				AllianceWallOutpost = new Pose2d(Inches.of(12), Inches.of(293),
						new Rotation3d(0, 0, 0).toRotation2d());
			} else if (current_alliance == DriverStation.Alliance.Red) {
				HubPose = new Pose2d(Inches.of(469.11), Inches.of(158.84),
						new Rotation3d(0, 0, 0).toRotation2d());
				AllianceWallDepot = new Pose2d(Inches.of(624.22), Inches.of(24),
						new Rotation3d(0, 0, 0).toRotation2d());
				AllianceWallOutpost = new Pose2d(Inches.of(624.22), Inches.of(293),
						new Rotation3d(0, 0, 0).toRotation2d());
			} 
		} else { //leaving in this edge case for testing purposes, but this should never be the case on the field
				HubPose = new Pose2d(Inches.of(182.11), Inches.of(158.84),
						new Rotation3d(0, 0, 0).toRotation2d());
				AllianceWallDepot = new Pose2d(Inches.of(12), Inches.of(24),
						new Rotation3d(0, 0, 0).toRotation2d());
				AllianceWallOutpost = new Pose2d(Inches.of(12), Inches.of(293),
						new Rotation3d(0, 0, 0).toRotation2d());
		}

		// Named Command Setup for Auto
		NamedCommands.registerCommand("ShootOnTheMoveCommand",
				new ShotCalculatorCommand(drivebase::getPose,
						drivebase::getFieldVelocity,
						drivebase::getRobotVelocity,
						HubPose,
						SuperStructure.TurretSubsytem,
						SuperStructure.FlywheelSubsystem,
						10.0)); // auto mode: exits after 5s or when turret + flywheel are on target

		// Non-blocking auton SOTM: one instance per target.
		// Start commands fire instantly (runOnce) and return immediately — path following
		// and all other auton actions continue unblocked in parallel.
		// ShotCalculatorStop cancels all three instances at once, triggering end() on
		// whichever one is running so hardware is always stopped cleanly.
		ShotCalculatorCommand autonSotmGoal = new ShotCalculatorCommand(
				drivebase::getPose,
				drivebase::getFieldVelocity,
				drivebase::getRobotVelocity,
				() -> HubPose,
				SuperStructure.TurretSubsytem,
				SuperStructure.FlywheelSubsystem,
				SuperStructure.kickerSubsystem,
				SuperStructure.BellyRollerSubsystem);
		ShotCalculatorCommand autonSotmOutpost = new ShotCalculatorCommand(
				drivebase::getPose,
				drivebase::getFieldVelocity,
				drivebase::getRobotVelocity,
				() -> AllianceWallOutpost,
				SuperStructure.TurretSubsytem,
				SuperStructure.FlywheelSubsystem,
				SuperStructure.kickerSubsystem,
				SuperStructure.BellyRollerSubsystem);
		ShotCalculatorCommand autonSotmDepot = new ShotCalculatorCommand(
				drivebase::getPose,
				drivebase::getFieldVelocity,
				drivebase::getRobotVelocity,
				() -> AllianceWallDepot,
				SuperStructure.TurretSubsytem,
				SuperStructure.FlywheelSubsystem,
				SuperStructure.kickerSubsystem,
				SuperStructure.BellyRollerSubsystem);

		NamedCommands.registerCommand("ShotCalculatorStart_Goal",
				Commands.runOnce(autonSotmGoal::schedule));
		NamedCommands.registerCommand("ShotCalculatorStart_Outpost",
				Commands.runOnce(autonSotmOutpost::schedule));
		NamedCommands.registerCommand("ShotCalculatorStart_Depot",
				Commands.runOnce(autonSotmDepot::schedule));

		// Stop cancels all three — safe to call even if only one (or none) is running.
		NamedCommands.registerCommand("ShotCalculatorStop", Commands.runOnce(() -> {
			autonSotmGoal.cancel();
			autonSotmOutpost.cancel();
			autonSotmDepot.cancel();
		}));
		NamedCommands.registerCommand("DeployHopper", SuperStructure.SetHopperPos());
		NamedCommands.registerCommand("SetHopperPosAgitate", SuperStructure.SetHopperPosAgitate());
		NamedCommands.registerCommand("RetractHopper", SuperStructure.SetHopperPosZero());
		NamedCommands.registerCommand("IntakeRollerOn", SuperStructure.SetIntakePWR(-0.8));
		NamedCommands.registerCommand("BellyFeed", SuperStructure.SetKickerAndBelly());
		NamedCommands.registerCommand("IntakeRollerOff", SuperStructure.SetIntakePWR(0));
		NamedCommands.registerCommand("Shake", new ShakeCommand(drivebase));
	}


	// Button Bindings for Drive and Operator controllers
	private void configureBindings() {

		Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
		drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

		// Operator Controls
		// Intake Rollers
		OperatorController.leftTrigger().whileTrue(SuperStructure.SetIntakePWR(-0.8))
				.onFalse(SuperStructure.SetIntakePWR(0));

		// Hood Homeing
		// OperatorController.a().onTrue(SuperStructure.HoodSubsystem.homing());

		// Kicker and Belly Control
		OperatorController.rightTrigger().whileTrue(SuperStructure.SetKickerAndBelly())
				.onFalse(SuperStructure.SetKickerAndBellyOff());

		OperatorController.x().whileTrue(SuperStructure.BackDriveKicker())
				.onFalse(SuperStructure.BackDriveKickeroff());

		// Hopper Extender Control
		OperatorController.leftBumper().onTrue(SuperStructure.SetHopperExtenderPower(0.3))
				.or(OperatorController.rightBumper().onTrue(SuperStructure.SetHopperExtenderPower(-0.5)))
				.onFalse(SuperStructure.SetHopperExtenderPower(0));

		OperatorController.a().onTrue(Commands.run(() -> {
			SuperStructure.SetIntakePWR(0.8);
			SuperStructure.FlywheelSubsystem.setVelocity(RPM.of(2000));
			SuperStructure.SetKickerAndBellyReverse();
		})).onFalse(Commands.run(() -> {
			SuperStructure.SetIntakePWR(0);
			SuperStructure.FlywheelSubsystem.setVelocity(RPM.of(0));
			SuperStructure.SetKickerAndBellyOff();
		}));

		OperatorController.back().onTrue(Commands.run(() -> {
			if(current_alliance == DriverStation.Alliance.Blue){
				current_alliance = DriverStation.Alliance.Red; 
			} 
			if(current_alliance == DriverStation.Alliance.Red){
				current_alliance = DriverStation.Alliance.Blue; 
			} 
		}));
		

		// SOTM - one shared command reads sotmTarget via supplier so only one instance
		// ever runs. toggleOnTrue ensures pressing a second direction cancels the first
		// (shared requirements), and pressing the same direction again turns it off.
		// SOTM now directly requires and controls kicker + hopper roller subsystems,
		// so the scheduler properly handles conflicts with right trigger bindings.
		// shakeCommand is declared here so it can be passed to SOTM for isScheduled() checks.
		ShakeCommand shakeCommand = new ShakeCommand(drivebase);

		// Helper to build a fresh SOTM command for each D-pad binding.
		// Each binding MUST have its own command instance — WPILib forbids reusing
		// a command that has already been composed into another group.
		// java.util.function.Supplier<ShotCalculatorCommand> makeSotm = () -> new ShotCalculatorCommand(drivebase::getPose,
																		//  drivebase::getFieldVelocity,
																		//  drivebase::getRobotVelocity,
																		//  () -> sotmTarget,
																		//  SuperStructure.TurretSubsytem,
																		//  SuperStructure.FlywheelSubsystem);
java.util.function.Supplier<ShotCalculatorCommand> makeSotm = () ->
	new ShotCalculatorCommand(
		drivebase::getPose,
		drivebase::getFieldVelocity,
		drivebase::getRobotVelocity,
		() -> sotmTarget,
		SuperStructure.TurretSubsytem,
		SuperStructure.FlywheelSubsystem,
		SuperStructure.kickerSubsystem,
		SuperStructure.BellyRollerSubsystem
	);

		// Store composed SOTM commands so povUp can cancel the whole composition.
		Command sotmDown  = Commands.runOnce(() -> sotmTarget = HubPose).andThen(makeSotm.get());
		Command sotmLeft  = Commands.runOnce(() -> sotmTarget = AllianceWallDepot).andThen(makeSotm.get());
		Command sotmRight = Commands.runOnce(() -> sotmTarget = AllianceWallOutpost).andThen(makeSotm.get());

		OperatorController.povDown().toggleOnTrue(sotmDown);
		OperatorController.povLeft().toggleOnTrue(sotmLeft);
		OperatorController.povRight().toggleOnTrue(sotmRight);

		// POV Up cancels any running SOTM composed command directly,
		// which properly triggers end() to stop the feed, flywheel, and turret.
		OperatorController.povUp().onTrue(Commands.runOnce(() -> {
			edu.wpi.first.wpilibj2.command.CommandScheduler scheduler =
					edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance();
			if (sotmDown.isScheduled())  scheduler.cancel(sotmDown);
			if (sotmLeft.isScheduled())  scheduler.cancel(sotmLeft);
			if (sotmRight.isScheduled()) scheduler.cancel(sotmRight);
		}));

		// OperatorController.povLeft().whileTrue(ShootOnTheMoveCommand.rebuildFromDashboard());

		/* ~~~~~~~~~~~~~~~~~~Drive Control~~~~~~~~~~~~~~~~~~~~~~~~ */
		if (ManualControl == true) {
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
		DriveController.leftTrigger().whileTrue(shakeCommand);

		// This is our boost control Right Trigger 
		//REVIEW - Does this limit the max speed to 80%?
		DriveController.axisGreaterThan(3, 0.01).onChange(Commands.runOnce(() -> {
			driveAngularVelocity.scaleTranslation(DriveController.getRightTriggerAxis() +
					0.35);
			driveAngularVelocity.scaleRotation((DriveController.getRightTriggerAxis() *
					Constants.MiscConstants.RotationSpeedScale) + 0.25);
		}).repeatedly()).whileFalse(Commands.runOnce(() -> {
			driveAngularVelocity.scaleTranslation(0.25);
			driveAngularVelocity.scaleRotation(0.15);
		}).repeatedly());
	}

	// Gets the selected autonomous command.
	// Called after gameInit() so NamedCommands are already registered.
	public Command getAutonomousCommand() {
		String selected = m_chooser.getSelected();
		if (selected == null) return Commands.none();
		return drivebase.getAutonomousCommand(selected);
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(false);
	}
}