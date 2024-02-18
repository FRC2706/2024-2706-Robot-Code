package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import static frc.robot.Config.ShooterConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkMax m_motor;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;

    private static Shooter shooter;
    public static Shooter getInstance() {
        if (shooter == null)
            shooter = new Shooter();

        return shooter;
    }

    // add Tunable Number later for ajusting Rpm on the flywheels
    public Shooter() {
        System.out.println("[Init] Creating Shooter");
        m_motor.restoreFactoryDefaults();
        
        m_motor.setCANTimeout(500); //Units in miliseconds
        m_motor.setIdleMode(IdleMode.kCoast);
        m_motor.setInverted(false);

        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        m_encoder.setAverageDepth(2);//check if 1 would work better
        
        //Voltage compensation
        m_motor.enableVoltageCompensation(12); 
        m_motor.setSmartCurrentLimit(40);  
        setBrake(false);

        //Smart motion parameters 
        m_pidController.setOutputRange(kMinOutput, kMinOutput);
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    public void setSetPoint(double setPoint) {
        m_pidController.setReference(setPoint, ControlType.kVelocity);
    }

    public void setSetVolt(double setVolt) {
        m_motor.setVoltage(setVolt);
    }

    public void setBrake(boolean enableBreak){
        m_motor.setIdleMode(enableBreak ? IdleMode.kBrake: IdleMode.kCoast);
    }

}
