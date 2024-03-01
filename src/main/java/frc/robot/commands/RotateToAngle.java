// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateToAngle extends TeleopSwerve {
  private Rotation2d m_givenAngle;
  private Rotation2d m_desiredAngle;

  /** Creates a new RotateAngleToVision. */
  public RotateToAngle(
      CommandXboxController driver,
      Rotation2d angle) {
    super(driver);
    
    m_givenAngle = angle;
    m_desiredAngle = SwerveSubsystem.rotateForAlliance(m_givenAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    m_desiredAngle = SwerveSubsystem.rotateForAlliance(m_givenAngle);
    SwerveSubsystem.getInstance().resetDriveToPose();
  }

  @Override
  protected double calculateRotationVal() {
    return SwerveSubsystem.getInstance().calculateRotation(m_desiredAngle);
  }
}
