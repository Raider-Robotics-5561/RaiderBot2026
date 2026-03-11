package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax climberMotor;

    public ClimberSubsystem () {

    // Set up the climb motor as a brushless motor
    climberMotor = new SparkMax(34, MotorType.kBrushless);

    SparkMaxConfig climbConfig = new SparkMaxConfig();
    // climbConfig.voltageCompensation(12);
    // climbConfig.smartCurrentLimit(40);
    climbConfig.idleMode(IdleMode.kBrake);
    
    climberMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    /**
     * Use to run the climber, can be set to run from 100% to -100%.
     * Keep in mind that the direction changes based on which way the winch is wound.
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runClimber(double speed){
        climberMotor.set(speed);
    }
}