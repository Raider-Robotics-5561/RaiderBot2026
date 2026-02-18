package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HopperSysytem.HopperExtenderSubsystem;
import frc.robot.subsystems.HopperSysytem.HopperRollerSubsystem;
import frc.robot.subsystems.HopperSysytem.IntakeSubsystem;
import frc.robot.subsystems.TurretSystem.FlywheelSubsystem;
import frc.robot.subsystems.TurretSystem.HoodSubsystem;
import frc.robot.subsystems.TurretSystem.TurretSubsystem;

public class SuperStructure extends SubsystemBase {
	private final FlywheelSubsystem 	  FlywheelSubsystem 	  = new FlywheelSubsystem();
	private final HoodSubsystem 		  HoodSubsystem 		  = new HoodSubsystem();
	private final TurretSubsystem 		  TurretSubsytem 		  = new TurretSubsystem();
	private final IntakeSubsystem 		  IntakeSubsystem         = new IntakeSubsystem();
	private final HopperExtenderSubsystem HopperExtenderSubsystem = new HopperExtenderSubsystem();
	private final ClimberSubsystem 		  ClimberSubsystem		  = new ClimberSubsystem();
	private final HopperRollerSubsystem   HopperRollerSubsystem   = new HopperRollerSubsystem();

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

	public Command SetHoodandFlywheelmin() {
	return
	FlywheelSubsystem.setVelocity(RPM.of(-1500)).alongWith(HoodSubsystem.setAngle(Rotations.of(2)));
	}

	public Command SetHoodandFlywheelIntake() {
	return
	FlywheelSubsystem.setVelocity(RPM.of(-5000)).alongWith(HoodSubsystem.setAngle(Rotations.of(1)));
	}

	public Command SetHoodandFlywheelmax() {
	return
	FlywheelSubsystem.setVelocity(RPM.of(-6000)).alongWith(HoodSubsystem.setAngle(Rotations.of(0)));
	}
	public Command SetAllMid() {
	return
	FlywheelSubsystem.setVelocity(RPM.of(-4250)).alongWith(HoodSubsystem.setAngle(Rotations.of(0.6)));
	}

	public Command StopHoodandFlywheel() {
	return FlywheelSubsystem.setVelocity(RPM.of(0))
	.alongWith(HoodSubsystem.setAngle(Rotations.of(0)))
	.alongWith(TurretSubsytem.setDutyCycle(0));
	}

	public Command SetTurretPWR(double power) {
	return TurretSubsytem.setDutyCycle(power);
	}

	public Command SetTurretPWRreverse() {
	return TurretSubsytem.setDutyCycle(-0.1);
	}

	public Command SetTurretPWRoff() {
	return TurretSubsytem.setDutyCycle(0);
	}

	public Command SetHopperRollers() {
		return HopperRollerSubsystem.setDutyCycle(0.3);
	}

	public Command SetHopperRollersoff() {
		return HopperRollerSubsystem.setDutyCycle(0.0);
	}

	//start hopper
	public Command SetHopperPos() {
		return HopperExtenderSubsystem.setHeight(Meters.of(0.1));
	}

	public Command SetHopperPosZero() {
		return HopperExtenderSubsystem.setHeight(Meters.of(-0.1));
	}
	//end hopper

	//Intake control
	public Command SetIntakePWR() {
		return IntakeSubsystem.setDutyCycle(-0.752);
	}

	public Command SetIntakePWRZero() {

		return IntakeSubsystem.setDutyCycle(0);
	}
	//end intake

	//Climb control
	public Command SetClimberPWRon() {

		return ClimberSubsystem.set(0.3);
	}

	public Command SetClimberPWRrev() {

		return ClimberSubsystem.set(-0.3);
	}
	//end climb

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
