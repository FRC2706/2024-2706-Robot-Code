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
    
    if (Config.Climber_CANID.CLIMBER != -1) {
      initializeSubsystem();
    }
    else
    {
      m_climber = null;
    }

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
      m_climber.setSmartCurrentLimit(40);
      
      ErrorTrackingSubsystem.getInstance().register(m_climber);

    }
      // Must be the last thing in the constructor
      burnFlash();
  }
      
  /**
   * Save the configurations from flash to EEPROM.
   */
  private void burnFlash() {
    try {
      Thread.sleep(200);
    } 
    catch (Exception e) {}

    m_climber.burnFlash();
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

