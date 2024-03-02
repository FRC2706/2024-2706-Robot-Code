// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  //Instance Variables
  private static ClimberSubsystem instance = null; // static object that contains all movement controls  
  private CANSparkMax m_climber;
  private RelativeEncoder m_encoder;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;
  public double currentPosition = 0;
  public boolean m_bGoodSensors = false;
  
  public static ClimberSubsystem getInstance() {
    if (instance == null) {
      SubsystemChecker.subsystemConstructed(SubsystemType.ClimberSubsystem);
      instance = new ClimberSubsystem();
    }
    return instance;
  }  

  /** Creates a new ClimberSubSystem. */
  private ClimberSubsystem() {
    initializeSubsystem();
  }

  private void initializeSubsystem() 
  {
    m_climber = new CANSparkMax(Config.Climber_CANID.CLIMBER, MotorType.kBrushless);

    if ( m_climber != null )
    {      
      m_bGoodSensors = true;
  
      // Factory Default to prevent unexpected behaviour
      m_climber.restoreFactoryDefaults();
      m_climber.setInverted(false);

      //Set maximum current
      m_climber.setSmartCurrentLimit(60);
    }
 }

    public boolean isAvailable() 
    {
        return m_climber != null;
    }

    //Run the climber

    public void StartClimberRPM(double percentOutput){
        m_climber.set(percentOutput);
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

