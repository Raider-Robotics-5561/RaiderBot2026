// package frc.robot.subsystems.HopperSysytem;

// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.DutyCycleOut;

//     public class CTRERollers extends SubsystemBase{

//     TalonFX Intake = new TalonFX(22);
// 	TalonFXConfiguration intakeconfig = new TalonFXConfiguration();

//     public CTRERollers() {
//         Intake.getConfigurator().apply(intakeconfig);
//     }

//      @Override
//     public void periodic() {
//         SmartDashboard.putNumber("IntakeMotorTemp", Intake.getDeviceTemp().getValueAsDouble());
//         SmartDashboard.putString("IntakeMotorAppliedControl", Intake.getAppliedControl().getControlInfo().toString());

//     }
    

//      public void setIntakePWR(double pwr) {
//         Intake.setControl(new DutyCycleOut(pwr).withEnableFOC(false));
//      }
    
// }
