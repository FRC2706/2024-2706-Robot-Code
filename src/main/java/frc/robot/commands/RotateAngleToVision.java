// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateAngleToVision extends TeleopSwerve {

  ProfiledPIDController pid = new ProfiledPIDController(5.0, 0, 0.4, 
                                        new TrapezoidProfile.Constraints(4 * Math.PI, 8 * Math.PI)); //pid to be tested
  SwerveSubsystem swerve;
  double angle;

  /** Creates a new RotateAngleToVision. */
  public RotateAngleToVision(
      SwerveSubsystem s_Swerve,
      CommandXboxController driver,
      double _angle) {
    super(s_Swerve, driver);
    
    this.angle = _angle;

    pid.setTolerance(0.1);
    pid.enableContinuousInput(-Math.PI, Math.PI);

    swerve = s_Swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    swerve.resetDriveToPose();
  }

  @Override
  protected double calculateRotationVal() {
    return(swerve.calculateRotation(SwerveSubsystem.getInstance().getHeading().getRadians(), this.angle));
  }
}
