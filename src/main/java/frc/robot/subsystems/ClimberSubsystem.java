// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Config;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  //Instance Variables
  private CANSparkMax m_climber;
  private RelativeEncoder m_encoder;
  //private double targetRPM = CLIMBER_RPM.getValue();
  public double kMaxOutput = 1;
  public double kMinOutput = -1;
  public double currentPosition = 0;
  public boolean m_bGoodSensors = false;
  private static final ClimberSubsystem INSTANCE_CLIMBER = new ClimberSubsystem();

  /** Creates a new ClimberSubSystem. */
  private ClimberSubsystem() {
    
    if (Config.Climber.CLIMBER != -1) {
      initializeSubsystem();
    }
    else
    {
      m_climber = null;
    }

  }

  private void initializeSubsystem() 
  {
    m_climber = new CANSparkMax(Config.Climber.CLIMBER, MotorType.kBrushless);

    if ( m_climber != null )
    {      
      m_bGoodSensors = true;
  
      // Factory Default to prevent unexpected behaviour
      m_climber.restoreFactoryDefaults();
      m_climber.setInverted(false);

      // PID controller for the Climber
      m_encoder = m_climber.getEncoder();

      REVLibError errorCode;
      
      //Use smart position closed loop controller
        
      //Set max acceleration, velocity, and minimum velocity
  
      
      // the following line cause the trouble: make m_bGoodSensors to false
      //  errorCode = m_pidController.setSmartMotionMinOutputVelocity(-1, 0);
      // m_bGoodSensors = m_bGoodSensors && (errorCode == REVLibError.kOk);

      //Set maximum current
      m_climber.setSmartCurrentLimit(60);
    }
 }

    public boolean isAvailable() 
    {
        return m_climber != null;
    }

     /*
     * Returns the singleton instance for the Climber Subsystem
     */
    public static ClimberSubsystem getInstance() {
      if ( INSTANCE_CLIMBER.isAvailable() == true)
        return INSTANCE_CLIMBER;
      else
        return null;
    }

    public void setClimberPosition()
    {
      if( m_bGoodSensors == true )
      { 
        //Get the current motor position from encoder
        currentPosition = m_encoder.getPosition();
      }
    }

    //Run the climber
    public void startClimber( int increPosition) {

      if( m_bGoodSensors == true )
      {
        //Use smartmotion to go from current position to new position

        //use the finxed incrementalPosition
        //m_pidController.setReference(currentPosition + incrementalPosition, ControlType.kSmartMotion, 0);

        //Use arbitrary feed forward: 0.5 voltage
        //note: arbitraryFF could be dependent on if there is already one cargo in the climber
        //m_pidController.setReference(currentPosition + incrementalPosition, ControlType.kSmartMotion, 0, 0.5, com.revrobotics.SparkMaxPIDController.ArbFFUnits.kVoltage);
      }
      else
      {
        //a constant speed, then only for one cargo
        //@todo: test this default speed
        m_climber.set(0.1);
      }

    }

    public void StartClimberRPM(double percentOutput){
      //if(m_bGoodSensors == true)
      //{
      //  m_pidController.setReference(targetRPM, ControlType.kVelocity, 1);
      //}
      //else
      //{
        m_climber.set(percentOutput);
      //}
    }

    public void stop() 
    {
      m_climber.stopMotor();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

