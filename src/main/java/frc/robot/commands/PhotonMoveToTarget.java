// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PhotonSubsystem;

//class
public class PhotonMoveToTarget extends Command {
  //declerations
  Translation2d targetOffset;
  boolean centerTarget;
  Rotation2d desiredHeading;
  double tolerance=0.01;


  public PhotonMoveToTarget(Translation2d _targetOffset, double _tolerance) {
    addRequirements(SwerveSubsystem.getInstance());
    addRequirements(PhotonSubsystem.getInstance());
    targetOffset = _targetOffset;
    centerTarget=true;
    tolerance=_tolerance;
  }

  //if you want a direction at the stopping point
  public PhotonMoveToTarget(Translation2d _targetOffset, Rotation2d _desiredHeading, double _tolerance) {
    addRequirements(SwerveSubsystem.getInstance());
    addRequirements(PhotonSubsystem.getInstance());
    targetOffset = _targetOffset;
    desiredHeading = _desiredHeading;
    centerTarget=false;
    tolerance=_tolerance;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveSubsystem.getInstance().resetDriveToPose();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d setPoint = PhotonSubsystem.getInstance().getTargetPos();
    Rotation2d rotationSetPoint = PhotonSubsystem.getInstance().getTargetRotation();
    if (centerTarget){
      SwerveSubsystem.getInstance().driveToPose(new Pose2d(setPoint.plus(targetOffset), rotationSetPoint));
    }else{
      SwerveSubsystem.getInstance().driveToPose(new Pose2d(setPoint.plus(targetOffset), desiredHeading));
    }
  }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SwerveSubsystem.getInstance().stopMotors();
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return SwerveSubsystem.getInstance().isAtPose(tolerance, tolerance*10);
  }
}
