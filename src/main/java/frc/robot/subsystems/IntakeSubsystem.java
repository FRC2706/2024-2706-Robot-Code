package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.Config;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class IntakeSubsystem extends SubsystemBase {

    private static final IntakeSubsystem INSTANCE_INTAKE = new IntakeSubsystem();

    private CANSparkMax m_intake;
    int targetSpeed = 0;
    double kMaxOutput = 1;
    double kMinOutput = -1;

    public boolean isActive() {
        return m_intake != null;
    }

    public static IntakeSubsystem getInstance() {
        if (INSTANCE_INTAKE.isActive())
            return INSTANCE_INTAKE;
        else
            return null;
    }


    private IntakeSubsystem() {
        if (Config.Intake.INTAKE != -1) {
            initializeSubsystem();
        } else {
            m_intake = null;
        }
    }

    private void initializeSubsystem() {
        
        m_intake = new CANSparkMax(Config.Intake.INTAKE, MotorType.kBrushless);
        m_intake.restoreFactoryDefaults();
        m_intake.setInverted(true);
        m_intake.setSmartCurrentLimit(70);

    }

    public void setMotorRPM(Double speed) {
        m_intake.set(speed);
    }

    public void setVoltage(double voltage){
        m_intake.setVoltage(voltage);
    }
    @Override
    public void periodic() {
        
    }

    public void stop()
    {
        m_intake.stopMotor();
    }

}