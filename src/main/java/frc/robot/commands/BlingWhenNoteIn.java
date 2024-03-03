// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BlingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class BlingWhenNoteIn extends Command {
  /** Creates a new BlingWhenNoteIn. */
  public BlingWhenNoteIn() {
    addRequirements(BlingSubsystem.getINSTANCE());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BlingSubsystem.getINSTANCE().setBrightness();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (IntakeSubsystem.getInstance().isBackSensorActive()) {
      BlingSubsystem.getINSTANCE().setOrange();
    } else {
      BlingSubsystem.getINSTANCE().setDisabled();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BlingSubsystem.getINSTANCE().setDisabled();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
