package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.HopperSysytem.HopperExtenderSubsystem;
import frc.robot.subsystems.HopperSysytem.HopperRollerSubsystem;
import frc.robot.subsystems.HopperSysytem.IntakeSubsystem;
import frc.robot.subsystems.HopperSysytem.KickerSubsystem;
import frc.robot.subsystems.TurretSystem.FlywheelSubsystem;
import frc.robot.subsystems.TurretSystem.HoodSubsystem;
import frc.robot.subsystems.TurretSystem.TurretSubsystem;

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

	public Command SetKickerAndBellyOff() {
	return
	//FlywheelSubsystem.setVelocity(RPM.of(0))
	//.alongWith(_HoodSubsystem.homing(Amps.of(20))
	HopperRollerSubsystem.setDutyCycle(0)
	.alongWith(kickerSubsystem.setDutyCycle(0));
	}

	public Command SetKickerAndBelly() {
	return
		kickerSubsystem.setDutyCycle(-0.9)
		.alongWith(new WaitCommand(1.0)
		.andThen(HopperRollerSubsystem.setVelocity(RPM.of(2500))).repeatedly());
	}

	public Command BackDriveKicker() {
	return
		kickerSubsystem.setDutyCycle(0.8);
	}

	public Command BackDriveKickeroff() {
	return
		kickerSubsystem.setDutyCycle(0);
	}
	
	//start hopper NEED TO FIX
	public Command SetHopperPosAgitate() {
		return HopperExtenderSubsystem.setHeight(Meters.of(20));
				
	}
	public Command SetHopperPos() {
		return HopperExtenderSubsystem.setHeight(Meters.of(30));			
	}
	public Command SetHopperPosZero() {
		return HopperExtenderSubsystem.setHeight(Meters.of(0))
									  .andThen(HopperExtenderSubsystem.stop());
	}
	//end hopper



	public Command SetHopperExtenderPower(double power) {

		return HopperExtenderSubsystem.set(power);
	}

	public Command SetIntakePWR(double power) {
		return IntakeSubsystem.setDutyCycle(power);
	}

}
