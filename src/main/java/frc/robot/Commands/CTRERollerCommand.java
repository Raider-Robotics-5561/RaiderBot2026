// package frc.robot.Commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.HopperSysytem.CTRERollers;

// public class CTRERollerCommand extends Command{
//     private final CTRERollers m_ctrerollers;

//   public CTRERollerCommand(CTRERollers ctrerollers) {
//     m_ctrerollers = ctrerollers;
//     addRequirements(ctrerollers);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_ctrerollers.setIntakePWR(-0.6);
//   }

//   // Called once the command ends or is interrupted.. Here we ensure the climber is not
//   // running once we let go of the button
//   @Override
//   public void end(boolean interrupted) {
//     m_ctrerollers.setIntakePWR(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }

// }
