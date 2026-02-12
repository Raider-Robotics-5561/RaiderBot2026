package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class HopperExtenderSubsystem extends SubsystemBase {
private final SparkMax HopperExtender = new SparkMax(14, MotorType.kBrushless);

 private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(0.00016541, 0, 0, RPM.of(500), RotationsPerSecondPerSecond.of(100))
            .withGearing(new MechanismGearing(3))
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("HopperExtender", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(3))
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.15))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController Hoppersmc = new SparkWrapper(HopperExtender, DCMotor.getNEO(1), motorConfig);

    private final PivotConfig HopperConfig = new PivotConfig(Hoppersmc)
            .withStartingPosition(Degrees.of(0))
            .withTelemetry("HopperMech", TelemetryVerbosity.HIGH)
            .withMOI(Meters.of(0.3048), Pounds.of(4)); // MOI Calculation
        

    private final Pivot Hopper = new Pivot(HopperConfig);

    public HopperExtenderSubsystem() {
    }

    public Command setAngle(Angle angle) {
        return Hopper.setAngle(angle);
    }



    public Command setAngle(Supplier<Angle> angleSupplier) {
        return Hopper.setAngle(angleSupplier);
    }

    public Angle getAngle() {
        return Hopper.getAngle();
    }

    public Command sysId() {
        return Hopper.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
    }

    @Override
    public void periodic() {
        Hopper.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        Hopper.simIterate();
    }
}
