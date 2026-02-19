package frc.robot.subsystems.HopperSysytem;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.MechanismGearing;




public class HopperExtenderSubsystem extends SubsystemBase {
	private final SparkMax HopperExtender = new SparkMax(33, MotorType.kBrushless);

	private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
      .withMechanismCircumference(Meters.of(Inches.of(2.16).in(Meters) * 18)) //- Fix to be correct
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

	private ElevatorConfig HopperConfig = new ElevatorConfig(Hoppersmc)
      .withStartingHeight(Meters.of(0))
      .withHardLimits(Meters.of(0), Meters.of(0.5))
      .withTelemetry("HopperExtender", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(3));

	private final Elevator Hopper = new Elevator(HopperConfig);

	public HopperExtenderSubsystem() {
	}

	  /**
   * Set the height of the elevator and does not end the command when reached.
   * @param angle Distance to go to.
   * @return a Command
   */
  public Command setHeight(Distance height) { return Hopper.run(height);}
  
  /**
   * Set the height of the elevator and ends the command when reached, but not the closed loop controller.
   * @param angle Distance to go to.
   * @return A Command
   */
  // public Command setHeightAndStop(Distance height) {
	//  return Hopper.runTo(height);
	// }
  
  /**
   * Set the elevators closed loop controller setpoint.
   * @param angle Distance to go to.
   */
  public void setHeightSetpoint(Distance height) { Hopper.setMeasurementPositionSetpoint(height);}

  /**
   * Move the elevator up and down.
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) { return Hopper.set(dutycycle);}

  /**
   * Run sysId on the {@link Elevator}
   */
  public Command sysId() { return Hopper.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Hopper.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Hopper.simIterate();
  }
}
