package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.TunableNumber;
import frc.robot.Config;

public class Shooter extends SubsystemBase {
    private CANSparkMax m_motor;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;

    private TunableNumber kP = new TunableNumber("Shooter/kP", Config.ShooterConstants.kP);
    private TunableNumber kI = new TunableNumber("Shooter/kI", Config.ShooterConstants.kI);
    private TunableNumber kD = new TunableNumber("Shooter/kD", Config.ShooterConstants.kD);
    private TunableNumber kFF = new TunableNumber("Shooter/kFF", Config.ShooterConstants.kFF);
 
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

        m_pidController.setOutputRange(Config.ShooterConstants.kMinOutput, Config.ShooterConstants.kMaxOutput);
        setPIDGains(kP.get(), kI.get(), kD.get());
        setFFGains(kFF.get());
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

    private void setPIDGains(double kP, double kI, double kD){
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
    }   

    private void setFFGains(double kFF){
        m_pidController.setFF(kFF);
    }   

    @Override
    public void periodic() {
        TunableNumber.ifChanged(hashCode(), ()->setPIDGains(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        TunableNumber.ifChanged(hashCode(), ()->setFFGains(kFF.get()), kFF);
    }
}
