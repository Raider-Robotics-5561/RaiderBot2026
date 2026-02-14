package frc.robot.subsystems;

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

public class IntakeSubsystem extends SubsystemBase {
	TalonFX Intake = new TalonFX(15);

	private SmartMotorControllerConfig IntakeConfig = new SmartMotorControllerConfig(this)
			.withControlMode(ControlMode.CLOSED_LOOP)
			.withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
			.withSimClosedLoopController(1, 0, 0)
			.withStatorCurrentLimit(Amps.of(40))
			.withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
			.withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
			.withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
			.withGearing(new MechanismGearing(2))
			.withMotorInverted(false)
			.withIdleMode(MotorMode.COAST)
			.withControlMode(ControlMode.CLOSED_LOOP)
			.withStatorCurrentLimit(Amps.of(40));

	// Create our SmartMotorController from our Spark and config with the NEO.
	private final SmartMotorController IntakeMotor = new TalonFXWrapper(Intake, DCMotor.getKrakenX60(1), IntakeConfig);

	private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(IntakeMotor)
			// Diameter of the flywheel.
			.withDiameter(Inches.of(2))
			// Mass of the flywheel.
			.withMass(Pounds.of(1))
			// Maximum speed of the shooter.
			.withSoftLimit(RPM.of(-5000), RPM.of(5000))
			// Telemetry name and verbosity for the arm.
			.withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

	// Shooter Mechanism
	private FlyWheel shooter = new FlyWheel(flywheelConfig);

	public AngularVelocity getVelocity() {
		return shooter.getSpeed();
	}

	public Command setVelocity(AngularVelocity speed) {
		return shooter.setSpeed(speed);
	}

	public Command setDutyCycle(double dutyCycle) {
		return shooter.set(dutyCycle);
	}

	public Command setVelocity(Supplier<AngularVelocity> speed) {
		return shooter.setSpeed(speed);
	}

	public Command setDutyCycle(Supplier<Double> dutyCycle) {
		return shooter.set(dutyCycle);
	}

	public Command sysId() {
		return shooter.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
	}

	@Override
	public void periodic() {
		shooter.updateTelemetry();
	}

	@Override
	public void simulationPeriodic() {
		shooter.simIterate();
	}
}