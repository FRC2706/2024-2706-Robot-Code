// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.Shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;


/** Add your docs here. */
public class ShooterIOSparkMax implements ShooterIO{
    private CANSparkMax m_motor = new CANSparkMax(32, MotorType.kBrushless);
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    
    private boolean setReverse = false;
    private double feedForward = 0;

    //TODO: add the spark max brun manager
    public ShooterIOSparkMax(){
        System.out.println("[Init] Creating ShooterIOSparkMax");
        m_motor.restoreFactoryDefaults();
        
        //Soft limit params
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);  
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        m_motor.setInverted(setReverse);
        //m_motor.setCANTimeout(500); //Units in miliseconds
        m_motor.setIdleMode(IdleMode.kCoast);

        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        m_encoder.setAverageDepth(2);//check if 1 would work better
        
        //Voltage compensation
        m_motor.enableVoltageCompensation(12); //Check this values
        m_motor.setSmartCurrentLimit(40); //Units in AMPS, should not be more than 40 check with 
        //Hardware to which port on the pdh the motor is connected to
        
        setShooterBrakeMode(false);

        //Smart motion parameters 
        m_pidController.setOutputRange(-1, 1);
        m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        m_pidController.setSmartMotionMaxVelocity(2000, 0);//check this values
        m_pidController.setSmartMotionMinOutputVelocity(0, 0);//check this values
        m_pidController.setSmartMotionMaxAccel(1500, 0);
        m_pidController.setSmartMotionAllowedClosedLoopError(0, 0);//check this values
    }
    
    @Override
    public void updateValues(ShooterIOValues values) {
        values.flywheelAppliedVolts = m_motor.getAppliedOutput();
        values.flywheelOutputCurrent = m_motor.getOutputCurrent();
        values.flywheelPositionRotations = m_encoder.getPosition();
        values.flywheelVelocityRPM = m_encoder.getVelocity();

    }

    @Override
    public void setCharacterizationVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public void setVoltage(double voltage){
        m_motor.setVoltage(voltage);
    }

    @Override
    public void setFF(double volts) {
        feedForward = volts;
    }

    @Override
    public void setPID(double p, double i, double d) {
        m_pidController.setP(p);
        m_pidController.setI(i);
        m_pidController.setD(d);
    }

    @Override
    public void setRPM(double rpm) {
        m_pidController.setReference(rpm, ControlType.kVelocity, 0, feedForward);
    }

    @Override
    public void setShooterBrakeMode(boolean enabled) {
        m_motor.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void stop() {
        m_pidController.setReference(0, ControlType.kVelocity);
    }
}
