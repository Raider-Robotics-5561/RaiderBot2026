package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.ClimberDownCommand;
import frc.robot.Commands.ClimberUpCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.SuperStructure;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
	private final SuperStructure SuperStructure = new SuperStructure();


	public final ClimberSubsystem m_climber = new ClimberSubsystem();
  
	final CommandXboxController DriveController = new CommandXboxController(0);
	final CommandXboxController OpController = new CommandXboxController(1);
	// The robot's subsystems and commands are defined here...
	private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
			"swerve"));

	private SendableChooser<Command> autoChooser;

	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
			() -> DriveController.getLeftY() * -1,
			() -> DriveController.getLeftX() * -1)
			// .withControllerRotationAxis(() -> DriveController.getRawAxis(2))
			.withControllerRotationAxis(() -> DriveController.getRightX() * -1)
			.deadband(Constants.MiscConstants.DEADBAND)
			.scaleTranslation(0.20)
			.scaleRotation(0.35)
			.allianceRelativeControl(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a
	 * fieldRelative input stream.
	 * //
	 */
	SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
			.withControllerHeadingAxis(DriveController::getLeftX,
					DriveController::getLeftY)
			.headingWhile(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a
	 * robotRelative input stream.
	 */
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
			.withControllerHeadingAxis(() -> Math.sin(
					DriveController.getRawAxis(
							// driverJoystick.getRawAxis(
							2) *
							Math.PI)
					*
					(Math.PI *
							2),
					() -> Math.cos(
							DriveController.getRawAxis(
									// driverJoystick.getRawAxis(
									2) *
									Math.PI)
							*
							(Math.PI *
									2))
			.headingWhile(true);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);

		// ======================================================================================

	}

	private void configureBindings() {

		Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

		Command driveFieldOrientedDirectAngleKeyboard =
		drivebase.driveFieldOriented(driveDirectAngleKeyboard);

		if (RobotBase.isSimulation())
		{
		drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
		} else
		{
		drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
		}

		// if (Robot.isSimulation())
		// {
		// DriveController.start().onTrue(Commands.runOnce(() ->
		// drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
		// DriveController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

		// REV 11--1817

		/* ~~~~~~~~~~~~~~~~~~Drive Control~~~~~~~~~~~~~~~~~~~~~~~~ */
         DriveController.start().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
		// DriveController.leftBumper().whileTrue(new ClimberUpCommand(m_climber));
     	// DriveController.rightBumper().whileTrue(new ClimberDownCommand(m_climber));

		DriveController.b().onTrue(SuperStructure.SetIntakePWR(-0.8)); // This should should be neative to run the intake in
		DriveController.y().onTrue(SuperStructure.SetIntakePWR(0));
		// DriveController.povLeft().onTrue(SuperStructure.SetTurretangle(0.4));
		// DriveController.povRight().onTrue(SuperStructure.SetTurretangle(-0.4));


		DriveController.a().onTrue(SuperStructure.SetAllMid());	
		DriveController.x().onTrue(SuperStructure.SetHoodandFlywheelZero());

		// DriveController.leftBumper().onTrue(SuperStructure.SetHopperPos());
		DriveController.leftBumper().onTrue(SuperStructure.SetHopperExtenderPower(0.3)).or(DriveController.rightBumper().onTrue(
			SuperStructure.SetHopperExtenderPower(-0.3)
		)).whileFalse(
			SuperStructure.SetHopperExtenderPower(0)
		);


		DriveController.povLeft().onTrue(SuperStructure.SetTurretPWR(0.2)).or(DriveController.povRight().onTrue(
			SuperStructure.SetTurretPWR(-0.2)
		)).whileFalse(
			SuperStructure.SetTurretPWR(0)
		);

		DriveController.povUp().onTrue(SuperStructure.SetHoodPWR(0.1)).or(DriveController.povDown().onTrue(
			SuperStructure.SetHoodPWR(-0.1)
		)).whileFalse(
			SuperStructure.SetHoodPWR(0)
		);


	// This is our boost control Right Trigger
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


	


	public void setupAutonomous() {
		// Named Commands go here
		// NamedCommands.registerCommand("GUI NAME", theCommand());
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(autoChooser);
	}

	/**
	 * Gets the selected autonomous command.
	 *
	 * @return the selected {@link Command}.
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void setMotorBrake(boolean brake)
	{
	drivebase.setMotorBrake(false);
	}
}