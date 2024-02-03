package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Shooter extends SubsystemBase {

    private CANSparkMax m_motor;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;

    private double kP, kD, kFF, kMaxOutput, kMinOutput, maxRPM; 
    
        /// add Tunable Number later for ajusting Rpm on the flywheels 

        public Shooter() {  
            m_motor = new CANSparkMax(1, MotorType.kBrushless);

            m_motor.restoreFactoryDefaults();

            m_pidController = m_motor.getPIDController();

            m_encoder = m_motor.getEncoder();

            //confi
            kP = 6e-5;
            kD = 0;
            kFF = 0.000015;
            kMaxOutput = 1;
            kMinOutput = -1;
            maxRPM = 5700; 

            m_pidController.setP(kP);
            m_pidController.setD(kD);
            m_pidController.setFF(kFF);
            m_pidController.setOutputRange(kMaxOutput, kMinOutput);
        }

        public double getVelocity() {

                return m_encoder.getVelocity();

        }

        public void setSetPoint(double setPoint){

            
            m_pidController.setReference(setPoint, ControlType.kVelocity);

        }

}
