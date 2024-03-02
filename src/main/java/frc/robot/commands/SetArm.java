// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetArm extends Command {
  private DoubleSupplier angleDegree;

  /** Creates a new SetArm. */
  public SetArm(DoubleSupplier angleDegree) {
    this.angleDegree = angleDegree;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ArmSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmSubsystem.getInstance().setJointAngle(Math.toRadians(angleDegree.getAsDouble()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.getInstance().stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //keep running motor
    return false;
  }
}

