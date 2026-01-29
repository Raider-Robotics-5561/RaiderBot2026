package frc.robot.subsystems.Hooded_Turret_Shooter;


import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
  TalonFX hoodMotor = new TalonFX(12);
 public final Angle            hardLowerLimit     = Degrees.of(0);
 private final Angle           hardUpperLimit     = Degrees.of(40);
    private final SmartMotorControllerConfig hoodMotorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(0.00016541, 0, 0, RPM.of(2500), RotationsPerSecondPerSecond.of(500))
            .withGearing(new MechanismGearing(22.57))
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withSupplyCurrentLimit(Amps.of(3))
            .withMotorInverted(false) //NOTE - May need to fix based on direction of motor
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController hoodSMC = new TalonFXWrapper(hoodMotor, DCMotor.getKrakenX44(1), hoodMotorConfig);

    private final ArmConfig hoodConfig = new ArmConfig(hoodSMC)
            .withLength(Inches.of(6)).withMass(Pound.of(1))
            .withStartingPosition(Degrees.of(0))
            .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
            .withSoftLimits(Degrees.of(0), Degrees.of(35))
            .withHardLimit(Degrees.of(0), Degrees.of(40)); // The Hood can be modeled as an arm since it has a
                                                            // gravitational force acted upon based on the angle its in


    private final Arm hood = new Arm(hoodConfig);

    public HoodSubsystem() {
    }

    public Command setAngle(Angle angle) {
        return hood.setAngle(angle);
    }

    public Command setAngle(Supplier<Angle> angleSupplier) {
        return hood.setAngle(angleSupplier);
    }

    public Angle getAngle() {
        return hood.getAngle();
    }


/**
   * Reset the encoder to the lowest position when the current threshold is reached. Should be used when the hood
   * position is unreliable, like startup. Threshold is only detected if exceeded for 0.1 seconds, and the motor moves
   * less than 0.2 degrees per second.
   *
   * @param threshold The current threshold held when the hood is at it's hard limit.
   * @return Command which resets our encoder
   */
  public Command homing(Current threshold)
  {
    Debouncer       currentDebouncer  = new Debouncer(0.1); // Current threshold is only detected if exceeded for 0.1 seconds.
    Voltage         StopVolts         = Volts.of(0); // Volts required to run the mechanism down. Could be negative if the mechanism is inverted.
    Voltage         runVolts          = Volts.of(-1); // Volts required to run the mechanism down. Could be negative if the mechanism is inverted.
    Angle           limitHit          = hardLowerLimit;  // Limit which gets hit. Could be the lower limit if the volts makes the hood go down.

    return Commands.startRun(hoodSMC::stopClosedLoopController, // Stop the closed loop controller
                             () -> hoodSMC.setVoltage(runVolts)) // Set the voltage of the motor
                   .until(() -> currentDebouncer.calculate(hoodSMC.getStatorCurrent().gte(threshold)))

                   .finallyDo(() -> {
                    hoodSMC.setVoltage(StopVolts);
                    hoodSMC.setEncoderPosition(limitHit);
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
}