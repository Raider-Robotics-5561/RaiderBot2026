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

public class IntakeRollerSubsystem extends SubsystemBase {
	TalonFX Intake = new TalonFX(22);

	private SmartMotorControllerConfig IntakeConfig = new SmartMotorControllerConfig(this)
			.withControlMode(ControlMode.CLOSED_LOOP)
			.withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
			.withSimClosedLoopController(1, 0, 0)
			.withStatorCurrentLimit(Amps.of(40))
			.withSupplyCurrentLimit(Amps.of(40))
			.withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
			.withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
			.withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
			.withGearing(new MechanismGearing(2))
			.withMotorInverted(false)
			.withIdleMode(MotorMode.COAST)
			.withControlMode(ControlMode.CLOSED_LOOP);
			
	private final SmartMotorController IntakeMotor = new TalonFXWrapper(Intake, DCMotor.getKrakenX60(1), IntakeConfig).disableFOC();

	private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(IntakeMotor)
			// Diameter of the flywheel.
			.withDiameter(Inches.of(2))
			// Mass of the flywheel.
			.withMass(Pounds.of(1))
			// Maximum speed of the shooter.
			.withSoftLimit(RPM.of(-5000), RPM.of(5000))
			// Telemetry name and verbosity for the arm.
			.withTelemetry("Intake", TelemetryVerbosity.HIGH);
			
	private FlyWheel Intakemotor = new FlyWheel(flywheelConfig);

	public AngularVelocity getVelocity() {
		return Intakemotor.getSpeed();
	}

	public Command setVelocity(AngularVelocity speed) {
		return Intakemotor.setSpeed(speed);
	}

	public Command setDutyCycle(double dutyCycle) {
		return Intakemotor.set(dutyCycle);
	}

	public Command setVelocity(Supplier<AngularVelocity> speed) {
		return Intakemotor.setSpeed(speed);
	}

	public Command setDutyCycle(Supplier<Double> dutyCycle) {
		return Intakemotor.set(dutyCycle);
	}

	public Command sysId() {
		return Intakemotor.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
	}

	@Override
	public void periodic() {
		Intakemotor.updateTelemetry();

		if(Intakemotor.getSpeed().baseUnitMagnitude() > 0) {
			SmartDashboard.putBoolean("Intake Status", false);
		} else {
			SmartDashboard.putBoolean("Intake Status", true);	
		}
	}

	@Override
	public void simulationPeriodic() {
		Intakemotor.simIterate();
	}
}