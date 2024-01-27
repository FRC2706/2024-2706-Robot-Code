package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.config.Config;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class IntakeSubsystem extends SubsystemBase {

    private IntakeSubsystem() {
        if.(Config.CANID.INTAKE != -1) {
            initializeSubsystem()
        } else {
            m_intake = null;
        }
    }

    private void initializeSubsystem() {
        
        m_intake = new CANSparkMax(Config.CANID.INTAKE, MotorType.kBrushless);

    }

    public void setMotorRPM(Double speed) {
        m_intake.set(speed)
    }

}