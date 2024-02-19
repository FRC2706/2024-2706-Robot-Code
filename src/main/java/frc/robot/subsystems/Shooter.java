package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.lib.lib2706.ErrorCheck.configureSpark;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;


public class Shooter extends SubsystemBase {
    
    private static Shooter shooter; 
    private double targetRPM;

    // network table entries
    private DoubleEntry m_shooterPSubs;
    private DoubleEntry m_shooterISubs;
    private DoubleEntry m_shooterDSubs;
    private DoubleEntry m_shooterIzSubs;
    private DoubleEntry m_shooterFFSubs;

    public static Shooter getInstance()
    {
        if (shooter == null)
            shooter = new Shooter();
 
        return shooter;
    }

    private CANSparkMax m_motor;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;

    private double kP, kD, kFF, kMaxOutput, kMinOutput, maxRPM; 
    
        // add Tunable Number later for ajusting Rpm on the flywheels 
        public Shooter() {  
            m_motor = new CANSparkMax(32, MotorType.kBrushless);

            m_motor.restoreFactoryDefaults();
            m_motor.setSmartCurrentLimit(70);
            m_pidController = m_motor.getPIDController();
            m_encoder = m_motor.getEncoder();

            NetworkTable ShooterTuningTable = NetworkTableInstance.getDefault().getTable("Shooter Tuning Table");
            m_shooterPSubs = ShooterTuningTable.getDoubleTopic("P").getEntry(Config.ShooterConfig.shooter_kP);
            m_shooterISubs = ShooterTuningTable.getDoubleTopic("I").getEntry(Config.ShooterConfig.shooter_kI);
            m_shooterDSubs = ShooterTuningTable.getDoubleTopic("D").getEntry(Config.ShooterConfig.shooter_kD);
            m_shooterIzSubs = ShooterTuningTable.getDoubleTopic("IZone").getEntry(Config.ShooterConfig.shooter_kIz);
            m_shooterFFSubs = ShooterTuningTable.getDoubleTopic("FF").getEntry(Config.ShooterConfig.shooter_kFF);

            updatePIDSettings();
            m_pidController.setOutputRange(Config.ShooterConfig.min_output, Config.ShooterConfig.max_output);
        }

        public void updatePIDSettings()
        {
            configureSpark("Shooter set FF", () -> (m_pidController.setFF(m_shooterFFSubs.get())));
            configureSpark("Shooter set P", () -> (m_pidController.setP(m_shooterPSubs.get())));
            configureSpark("Shooter set I", () -> (m_pidController.setI(m_shooterISubs.get())));
            configureSpark("Shooter set D", () -> (m_pidController.setD(m_shooterDSubs.get())));
            configureSpark("Shooter set Iz", () -> (m_pidController.setIZone(m_shooterIzSubs.get())));
        }

        public double getVelocity() {

            return m_encoder.getVelocity();
        }

        public void setSetRPM(double setPoint){

            
            m_pidController.setReference(setPoint, ControlType.kVelocity);

        }

        public void setSetVolt(double setVolt){
            
           m_motor.setVoltage(setVolt);

           //need to add a delay so the shooter can spin up, then the intake will lunch the note up into the shoorter part.
            
        }

        public void stopMotor()
        {
            m_motor.stopMotor();
        }

        public boolean isAtTargetRPM()
        {
            if (getVelocity() > (targetRPM - Config.ShooterConfig.TARGET_PRM_TOLERANCE))
                return true;       
            else
                return false; 
            
        }

        public void setTargetRPM(double targetPRM)
        {
            this.targetRPM = targetPRM;
        }
}
