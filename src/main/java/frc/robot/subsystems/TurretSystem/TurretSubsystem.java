package frc.robot.subsystems.TurretSystem;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Radians;



import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.motorcontrollers.SmartMotorControllerConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.config.SensorConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.motorcontrollers.simulation.Sensor;
import yams.motorcontrollers.SimSupplier;


public class TurretSubsystem extends SubsystemBase {
	TalonFX turretMotor = new TalonFX(9);
	CANdi candi = new CANdi(34);
	final DigitalInput m_forwardLimit = new DigitalInput(0);
	AbsoluteEncoderSubsystem abs_encoder = new AbsoluteEncoderSubsystem();
	TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
	CANdiConfiguration configs = new CANdiConfiguration();

	// // Use CANdi's Quadrature encoder as the motor's feedback sensor
	// talonConfigs.Feedback.withRemoteCANdiQuadrature(candi);

	// // Use CANdi's S1 input as a remote forward limit switch
	// talonConfigs.HardwareLimitSwitch.withForwardLimitRemoteCANdi(candi, S1);

	// talonMotor.getConfigurator().apply(talonConfigs);

	private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
			.withClosedLoopController(100, 5, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
			.withGearing(new MechanismGearing(35.56))
			.withIdleMode(MotorMode.BRAKE)
			.withMotorInverted(false)
			.withFeedforward(new SimpleMotorFeedforward(1, 0.01))
			// .withExternalEncoder(abs_encoder)
			// .withExternalEncoderInverted(false)
    	    // .withExternalEncoderGearing(1)
      		// .withExternalEncoderZeroOffset(Degrees.of(33.25))
     		// .withUseExternalFeedbackEncoder(true)
			.withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
			// Power Optimization
			.withStatorCurrentLimit(Amps.of(40))
			.withClosedLoopRampRate(Seconds.of(0.25))
			.withOpenLoopRampRate(Seconds.of(0.25))
			.withControlMode(ControlMode.CLOSED_LOOP);
	// .withContinuousWrapping(Rotations.of(0),Rotations.of(360));
	private final SmartMotorController turretSMC = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX44(1),
			motorConfig);
	private final PivotConfig turretConfig = new PivotConfig(turretSMC)
			.withStartingPosition(Degrees.of(abs_encoder.getAngleDegrees())) // Starting position of the Pivot
			// .withWrapping(Degrees.of(0), Degrees.of(360)) // Wrapping enabled bc the
			// pivot can spin
			// infinitely
			.withSoftLimits(Rotations.of(-0.4), Rotations.of(0.4))
			.withHardLimit(Rotations.of(-0.45), Rotations.of(0.45)) // Hard limit bc wiring prevents infinite
															// spinning
			.withTelemetry("TurretMech", TelemetryVerbosity.HIGH) // Telemetry
			.withMOI(Meters.of(0.25), Pounds.of(4)); // MOI Calculation

	private final Pivot turret = new Pivot(turretConfig);

	public TurretSubsystem() {

	}
  	// Robot to turret transform, from center of robot to turret.
  	private final Transform3d roboToTurret = new Transform3d(Feet.of(-1.5), Feet.of(0), Feet.of(0.5), Rotation3d.kZero);

	public Command setAngle(Angle angle) {
		return turret.setAngle(angle);
	}

	public Command setAngle(Supplier<Angle> angleSupplier) {
		return turret.setAngle(angleSupplier);
	}

	public Angle getAngle() {
		return turret.getAngle();
	}

	public Command sysId() {
		return turret.sysId(
				Volts.of(4.0), // maximumVoltage
				Volts.per(Second).of(0.5), // step
				Seconds.of(8.0) // duration
		);
	}

	public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
		return turret.set(dutyCycleSupplier);
	}

	public Command setDutyCycle(double dutyCycle) {
		return turret.set(dutyCycle);
	}

	@Override
	public void periodic() {
		turret.updateTelemetry();
		// SmartDashboard.putNumber("relative Angle Raw", getAngle());

	}

	@Override
	public void simulationPeriodic() {
		turret.simIterate();
	}
	public void setAngleSetpoint(Angle measure)
  {
    turret.setMechanismPositionSetpoint(measure);
  }

  public Pose2d getPose(Pose2d robotPose)
  {
    return robotPose.plus(new Transform2d(
        roboToTurret.getTranslation().toTranslation2d(), roboToTurret.getRotation().toRotation2d()));
  }

  public ChassisSpeeds getVelocity(ChassisSpeeds robotVelocity, Angle robotAngle)
  {
    var robotAngleRads = robotAngle.in(Radians);
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
          * (roboToTurret.getY() * Math.cos(robotAngleRads)
             - roboToTurret.getX() * Math.sin(robotAngleRads));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
          * (roboToTurret.getX() * Math.cos(robotAngleRads)
             - roboToTurret.getY() * Math.sin(robotAngleRads));

    return new ChassisSpeeds(turretVelocityX,
                             turretVelocityY,
                             robotVelocity.omegaRadiansPerSecond + turretSMC.getMechanismVelocity().in(RadiansPerSecond));
  }
}