package frc.robot.subsystems;

/*
 * ControlStructure.java
 *
 * Created on: Dec 20, 2025
 * Author: Chris
 *
 * What is this?
 *   This file is the command base file which will be fed into InputStructure.java 
 *
 * Changes:
 *  12-20-25 Base file created to build off of during the season
 */

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.AddressableLEDSubsystem.LEDModes;
// import frc.robot.subsystems.AddressableLEDSubsystem.LEDViews;



public class ControlStructure extends SubsystemBase {
    private final SwerveSubsystem swerve;


    /**
     * Initializer for the Control superstructure.
     * This is the "brain" that controls the body.
     * Calls for all subsystems.
     */
    public ControlStructure(SwerveSubsystem swerve) {
        this.swerve = swerve;

        if (RobotBase.isSimulation()) {
            return;
        }
    }

    @Override
    public void simulationPeriodic() {
        return;
    }


    /**
     * Runs a command that sets all LEDs to scrolling rainbow.
     *
     * @return a {@link Command} that sets all LEDs to scrolling rainbow.
     *
     * NOTE - Stole this code from team 5517. Will impliment LED's later once we get an Idea of our LED situation.
    private Command setLEDRainbow() {
        return Commands.run(() -> {
            led.runLED(LEDViews.BOTH, LEDModes.RAINBOW);
        }).finallyDo(() -> led.runLED(LEDViews.BOTH, LEDModes.OFF));
    }
*/

}
