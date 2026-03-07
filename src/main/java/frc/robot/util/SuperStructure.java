package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.*;


import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.HopperSysytem.HopperExtenderSubsystem;
import frc.robot.subsystems.HopperSysytem.HopperRollerSubsystem;
import frc.robot.subsystems.HopperSysytem.IntakeSubsystem;
import frc.robot.subsystems.HopperSysytem.KickerSubsystem;
import frc.robot.subsystems.TurretSystem.FlywheelSubsystem;
import frc.robot.subsystems.TurretSystem.HoodSubsystem;
import frc.robot.subsystems.TurretSystem.TurretSubsystem;
import frc.robot.util.ShooterTargetingSystem.Shot;

public class SuperStructure extends SubsystemBase {
	public final FlywheelSubsystem 	  FlywheelSubsystem 	  = new FlywheelSubsystem();
	public final HoodSubsystem 		  HoodSubsystem 		  = new HoodSubsystem();
 	public final TurretSubsystem 		  TurretSubsytem 		  = new TurretSubsystem();
	public final IntakeSubsystem 		  IntakeSubsystem         = new IntakeSubsystem();
	public final HopperExtenderSubsystem HopperExtenderSubsystem = new HopperExtenderSubsystem();
	public final HopperRollerSubsystem   HopperRollerSubsystem   = new HopperRollerSubsystem();
	public final KickerSubsystem 		  kickerSubsystem         = new KickerSubsystem();

	/**
	 * Initializer for the Control superstructure.
	 * This is the "brain" that controls the body.
	 * Calls for all subsystems.
	 */
	public SuperStructure() {

		if (RobotBase.isSimulation()) {
			return;
		}
	}

	@Override
	public void simulationPeriodic() {
		return;
	}

	// public Command SetHoodandFlywheelmin() {
	// return
	// FlywheelSubsystem.setVelocity(RPM.of(-1500));
	// //.alongWith(HoodSubsystem.setAngle(Rotations.of(2)));
	// }

	// public Command SetHoodandFlywheelZero() {
	// return
	// FlywheelSubsystem.setVelocity(RPM.of(0))
	// .alongWith(_HoodSubsystem.homing(Amps.of(20))
	// .alongWith(HopperRollerSubsystem.setDutyCycle(0))
	// .alongWith(kickerSubsystem.setDutyCycle(0)));
	// }

	// public Command SetAllMid() {
	// return
	// 	FlywheelSubsystem.setVelocity(RPM.of(0))//-3500))
	// 	.alongWith(_HoodSubsystem.setAngle(Rotations.of(0.0))
	// 	// .alongWith(HoodSubsystem.setAngle(Rotations.of(1)))
	// 	.andThen(kickerSubsystem.setDutyCycle(-0.8))
	// 	.alongWith(new WaitCommand(1)
	// 	.andThen(HopperRollerSubsystem.setDutyCycle(0.6)).repeatedly()));
	// }

	// public Command SetTurretPWR(double power) {
	// return TurretSubsytem.setDutyCycle(power);
	// }

	// public Command SetTurretangle(double angle) {
	// return TurretSubsytem.setAngle(Rotations.of(angle));
	// }

	// public Command SetHoodPWR(double power) {
	// return _HoodSubsystem.setDutyCycle(power);
	// }

	// public Command SetHoodAngle() {
	// 	return _HoodSubsystem.setAngle(Rotations.of(0.1));//Rotation.of(5));
	// }
	
	//start hopper NEED TO FIX
	public Command SetHopperPos() {
		return HopperExtenderSubsystem.setHeight(Meters.of(0.5));
	}

	public Command SetHopperPosZero() {
		return HopperExtenderSubsystem.setHeight(Meters.of(-0.3));
	}
	//end hopper

	//Intake control
	public Command SetTrueIntake() {
		return HopperExtenderSubsystem.setHeight(Meters.of(0.3))
			   .alongWith(IntakeSubsystem.setDutyCycle(-0.752))
			   .alongWith(HopperRollerSubsystem.setDutyCycle(0.8));
	}

	//end intake


	public Command SetHopperExtenderPower(double power) {

		return HopperExtenderSubsystem.set(power);
	}

	public Command SetIntakePWR(double power) {

		return IntakeSubsystem.setDutyCycle(power);
	}
	/**
	 * Runs a command that sets all LEDs to scrolling rainbow.
	 *
	 * @return a {@link Command} that sets all LEDs to scrolling rainbow.
	 *
	 *         NOTE - Stole this code from team 5517. Will impliment LED's later
	 *         once we get an Idea of our LED situation.
	 *         private Command setLEDRainbow() {
	 *         return Commands.run(() -> {
	 *         led.runLED(LEDViews.BOTH, LEDModes.RAINBOW);
	 *         }).finallyDo(() -> led.runLED(LEDViews.BOTH, LEDModes.OFF));
	 *         }
	 */

}
