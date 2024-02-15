// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinUntilIn extends Command {

  private IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
  private double targetSpeed;

  /** Creates a new SpinUntilIn. */
  public SpinUntilIn(double speed) {
    targetSpeed = speed;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ( intakeSubsystem != null ) {
      intakeSubsystem.setMotorRPM(targetSpeed);
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.isSwitch3True();
    }
}
