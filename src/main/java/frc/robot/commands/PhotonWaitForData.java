// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonSubsystem;

//class
public class PhotonWaitForData extends Command {

  int id = -1;
  public PhotonWaitForData(int desiredId) {
    addRequirements(PhotonSubsystem.getInstance());
    //get the id
    id = desiredId;
  }
  public PhotonWaitForData() {
    addRequirements(PhotonSubsystem.getInstance());
    id = -1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //reset the subsystem with the new id
    PhotonSubsystem.getInstance().reset(id);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //end when the subsystem has enough data
    return (PhotonSubsystem.getInstance().hasData());
  }
}
