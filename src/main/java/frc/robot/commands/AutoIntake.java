// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeStates;

public class AutoIntake extends Command {

  /** Creates a new AutoIntake. */
  public AutoIntake() {
    
    addRequirements(Intake.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Intake.getInstance().setMode(IntakeStates.Modes.STOP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intake.getInstance().allowAutoMovement();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
