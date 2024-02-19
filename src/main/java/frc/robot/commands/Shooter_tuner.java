// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shooter_tuner extends Command {

  //variable for setpoint
  private final Shooter shooter = Shooter.getInstance() ;
  private double setPoint;

  /** Creates a new Shooter_tuner. */
  public Shooter_tuner(double setPoint) { 

    this.setPoint = setPoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //set Voltage
    shooter.setSetVolt(setPoint);

    //set RPM
    //shooter.setSetRPM(1000);

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //shooter.setSetVolt(0);

    //++
    shooter.stopMotor();


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
