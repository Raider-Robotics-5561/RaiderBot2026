package frc.robot.util.TurretSystem;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.*;

public class HoodSubsystem extends SubsystemBase {
	final CANBus canbus = new CANBus("Turret");
	TalonFX hoodMotor = new TalonFX(12, canbus);
	public final Angle hardLowerLimit = Degrees.of(0);
	public static final Angle softLimitMin = Degrees.of(0);
	public static final Angle softLimitMax = Degrees.of(35);
	public Angle HoodAngle = Degrees.of(0);

	public final Current homeingCurrentThreshold = Amps.of(10); // Current threshold to detect when the hood has hit its hard limit during homing
	Debouncer currentDebouncer = new Debouncer(0.001); // Current threshold is only detected if exceeded for 0.1
	Voltage runVolts = Volts.of(-1); // Volts required to run the mechanism down. Could be negative if the mechanism

	private final SmartMotorControllerConfig hoodMotorConfig = new SmartMotorControllerConfig(this)
			.withClosedLoopController(650, 10,0, RPM.of(7000), RotationsPerSecondPerSecond.of(7000))
			.withFeedforward(new SimpleMotorFeedforward(8, 4))
			.withGearing(new MechanismGearing(533.25))
			.withIdleMode(MotorMode.BRAKE)
			.withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
			.withStatorCurrentLimit(Amps.of(40))
			.withSupplyCurrentLimit(Amps.of(3))
			.withMotorInverted(false) // NOTE - May need to fix based on direction of motor
			.withClosedLoopRampRate(Seconds.of(0.25))
			.withOpenLoopRampRate(Seconds.of(0.25))
			//.withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557)) // NOTE - Not Sure where these came form but I will keep them
			//.withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
			.withControlMode(ControlMode.CLOSED_LOOP);

	public final SmartMotorController hoodSMC = new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX44(1),
			hoodMotorConfig);

	private final ArmConfig hoodConfig = new ArmConfig(hoodSMC)
			.withLength(Inches.of(6)).withMass(Pound.of(1))
			.withStartingPosition(Degrees.of(0))
			.withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
			//.withSoftLimits(Degrees.of(0), Degrees.of(35))
			//HARD LIMIT IS FOR SIM
			.withHardLimit(Degrees.of(0), Degrees.of(30)); // The Hood can be modeled as an arm since it has a
															// gravitational force acted upon based on the angle its in

	private final Arm hood = new Arm(hoodConfig);

	public HoodSubsystem() {
	}

	public Command setAngle(Angle angle) {
		return hood.setAngle(angle);
	}

	public Command setAngleDashboard() {
		return hood.setAngle(Degrees.of(SmartDashboard.getNumber("Hood Angle Requested", 0)));
	}

	public Angle getAngle() {
		return hood.getAngle();
	}

	public Command homing() {
		return Commands.startRun(hoodSMC::stopClosedLoopController, // Stop the closed loop controller
				() -> hoodSMC.setVoltage(runVolts)) // Set the voltage of the motor
				.until(() -> currentDebouncer.calculate(hoodSMC.getStatorCurrent().gte(homeingCurrentThreshold)))

				.finallyDo(() -> {
					hoodSMC.setVoltage(Volts.of(0)); // Stop the motor
					hoodSMC.setEncoderPosition(hardLowerLimit);
					hoodSMC.startClosedLoopController();
				});
	}

	public Command sysId() {
		return hood.sysId(
				Volts.of(4.0), // maximumVoltage
				Volts.per(Second).of(0.5), // step
				Seconds.of(8.0) // duration
		);
	}

	public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
		return hood.set(dutyCycleSupplier);
	}

	public Command setDutyCycle(double dutyCycle) {
		return hood.set(dutyCycle);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("HoodSMC/Stator Current", hoodSMC.getStatorCurrent().baseUnitMagnitude());
		SmartDashboard.putNumber("HoodSMC/Mechanism Velocity", hoodSMC.getMechanismVelocity().baseUnitMagnitude());
		hood.updateTelemetry();
	}

	@Override
	public void simulationPeriodic() {
		hood.simIterate();
	}

	public void setAngleSetpoint(Angle measure)
  {
    hood.setMechanismPositionSetpoint(measure);
  }

   
}