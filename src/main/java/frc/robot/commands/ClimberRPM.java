// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberRPM extends CommandBase {

  private ClimberSubsystem climber;
  private double m_getPercentOutput;
  /** Creates a new IndexerCargo. */
  public ClimberRPM(double getPercentOutput) {

    climber = ClimberSubsystem.getInstance();
    
    // Use addRequirements() here to declare subsystem dependencies.
    if ( climber != null )
    {
      addRequirements(climber);
    }

    m_getPercentOutput = getPercentOutput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber == null){
      return;

      //System.out.println("auto mode: "+ Robot.m_bAutoMode);
      // System.out.println("indexer sensor: "+ indexer.m_bForIntakeGoodSensors);
    }
    else{
      climber.StartClimberRPM(m_getPercentOutput);
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    if ( climber != null )
      climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}

//for the case when the switch is at the end of the indexer, closer to shooter
//  // Called every time the scheduler runs while the command is scheduled.
//  @Override
//  public void execute() {
   
//    //@todo: only consider one detection, ignore all of the following detections.
//    if(bFirstSwitchDetected == false)
//    {
//      switchDetected = switchDetector.isDetected();
//      System.out.println("switch detected: "+switchDetected);
//    }

//    if (switchDetected == true) 
//    {
//      bFirstSwitchDetected = true;
//      indexer.stop();
//    } 
//    else 
//    {
//      colorSensorDetected = colorSensor.isDetected();
//      if ( colorSensorDetected == true)
//      {
//        System.out.println("detected one cargo");
//      }

//      if (colorSensorDetected == true && colorSensorFirstDetected == false) 
//      {
//        colorSensorFirstDetected = true;
//        System.out.println("start shuffling the cargo");
//      }
     
//      if (colorSensorFirstDetected == true) 
//      {
       
//        indexer.runForIntake();
//        counter++;
       
//        // TODO tune for 100
//        if (counter > 200) {
//          // At this time, the cargo should be in the indexer, and the indexer should stop

//          //reset for the next cargo 
//          colorSensorFirstDetected = false;
//          colorSensorDetected = false;
//          counter = 0;
//          indexer.setIndexerPosition();
//          System.out.println("stop shuffling the cargo");
//          indexer.stop();

//        }
//      } else {
//        indexer.stop();
//      }
//    }
//  }