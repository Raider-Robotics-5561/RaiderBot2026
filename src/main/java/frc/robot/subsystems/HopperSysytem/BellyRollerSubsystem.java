package frc.robot.subsystems.HopperSysytem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class BellyRollerSubsystem extends SubsystemBase {
	TalonFX HopperRoller = new TalonFX(24);
		//NOTE - Will need to update the CANid to fit the new kraken
	private SmartMotorControllerConfig HopperRollerConfig = new SmartMotorControllerConfig(this)
			.withControlMode(ControlMode.CLOSED_LOOP)
			.withClosedLoopController(1, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
			.withSimClosedLoopController(1, 0, 0)
			.withStatorCurrentLimit(Amps.of(40))
			.withSupplyCurrentLimit(Amps.of(40))
			.withFeedforward(new SimpleMotorFeedforward(0.5, 0.0001))
			.withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
			.withTelemetry("HopperRollerMotor", TelemetryVerbosity.HIGH)
			.withGearing(new MechanismGearing(1))
			.withMotorInverted(false)
			.withIdleMode(MotorMode.COAST)
			.withControlMode(ControlMode.CLOSED_LOOP);

	// Create our SmartMotorController from our Spark and config with the NEO.
	private final SmartMotorController HopperRollerMotor = new TalonFXWrapper(HopperRoller, DCMotor.getKrakenX60(1), HopperRollerConfig);

	private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(HopperRollerMotor)
			// Diameter of the flywheel.
			.withDiameter(Inches.of(2))
			// Mass of the flywheel.
			.withMass(Pounds.of(1))
			// Maximum speed of the shooter.
			.withSoftLimit(RPM.of(-5000), RPM.of(5000))
			// Telemetry name and verbosity for the arm.
			.withTelemetry("HopperRollers", TelemetryVerbosity.HIGH);

	// Shooter Mechanism
	private FlyWheel HopperRollers = new FlyWheel(flywheelConfig);

	public AngularVelocity getVelocity() {
		return HopperRollers.getSpeed();
	}

	public Command setVelocity(AngularVelocity speed) {
		return HopperRollers.setSpeed(speed);
	}

	public Command setDutyCycle(double dutyCycle) {
		return HopperRollers.set(dutyCycle);
	}

	/**
	 * Directly sets the hopper roller velocity without creating a Command.
	 * Use this from inside another command's execute() to avoid subsystem conflicts.
	 */
	public void setVelocityDirect(AngularVelocity speed) {
		HopperRollers.setMechanismVelocitySetpoint(speed);
	}

	/**
	 * Directly sets the hopper roller duty cycle without creating a Command.
	 * Use this from inside another command's execute() to avoid subsystem conflicts.
	 */
	public void setDutyCycleDirect(double dutyCycle) {
		HopperRoller.set(dutyCycle);
	}

	public Command setVelocity(Supplier<AngularVelocity> speed) {
		return HopperRollers.setSpeed(speed);
	}

	public Command setDutyCycle(Supplier<Double> dutyCycle) {
		return HopperRollers.set(dutyCycle);
	}

	public Command sysId() {
		return HopperRollers.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
	}

	@Override
	public void periodic() {
		HopperRollers.updateTelemetry();

		if(HopperRollers.getSpeed().baseUnitMagnitude() > 0) {
			SmartDashboard.putBoolean("HopperRollers Status", false);
		} else {
			SmartDashboard.putBoolean("HopperRollers Status", true);	
		}
	}

	@Override
	public void simulationPeriodic() {
		HopperRollers.simIterate();
	}
}