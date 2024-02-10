// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import frc.lib.lib2706.ProfiledPIDFFController;
import frc.robot.Config;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

/** Add your docs here. */
public class ArmIOSparkMax implements ArmIO{
  private CANSparkMax m_motor = new CANSparkMax(Config.CANID.ARM_SPARK_CAN_ID, MotorType.kBrushless); 
 // bottom SparkMax motor controller
  private SparkAbsoluteEncoder m_absoluteEncoder;
  private SparkPIDController m_pidController;

  public ArmIOSparkMax(){
    m_motor.restoreFactoryDefaults();
    m_motor.setSmartCurrentLimit(Config.ArmConfig.CURRENT_LIMIT);
    m_motor.setInverted(Config.ArmConfig.SET_INVERTED); // sets movement direction

    m_motor.setSoftLimit(SoftLimitDirection.kForward, Config.ArmConfig.arm_forward_limit);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, Config.ArmConfig.arm_reverse_limit);
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, Config.ArmConfig.SOFT_LIMIT_ENABLE);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, Config.ArmConfig.SOFT_LIMIT_ENABLE);

    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    m_absoluteEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
    m_absoluteEncoder.setInverted(Config.ArmConfig.INVERT_ENCODER);
    m_absoluteEncoder.setPositionConversionFactor(Config.ArmConfig.armPositionConversionFactor);
    m_absoluteEncoder.setVelocityConversionFactor(Config.ArmConfig.armVelocityConversionFactor);
    m_absoluteEncoder.setZeroOffset(Math.toRadians(Config.ArmConfig.armAbsEncoderOffset));

    m_pidController = m_motor.getPIDController();
    m_pidController.setFeedbackDevice(m_absoluteEncoder);

  }
  @Override
  public void updateValues(ArmIOValues inputs) {
    // TODO Auto-generated method stub
    ArmIO.super.updateValues(inputs);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    m_motor.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast); // sets brakes when there is no motion
  }

  @Override
  public void setFF(double ff) {

  }
  @Override
  public void setPID(double p, double i, double d) {
    // TODO Auto-generated method stub
    ArmIO.super.setPID(p, i, d);
  }
  @Override
  public void setSetpoint(double setpointRads) {
    // TODO Auto-generated method stub
    ArmIO.super.setSetpoint(setpointRads);
  }
  @Override
  public void setVolts(double volts) {
    // TODO Auto-generated method stub
    ArmIO.super.setVolts(volts);
  }
  @Override
  public void stop() {
    // TODO Auto-generated method stub
    ArmIO.super.stop();
  }

  //private ExponentialProfile  a;

}
