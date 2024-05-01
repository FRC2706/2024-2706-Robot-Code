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
import frc.robot.Config.PhotonConfig;
import frc.robot.subsystems.PhotonSubsystem;

//class
public class PhotonMoveToTarget extends Command {
  //declerations
  Translation2d targetOffset;
  boolean centerTarget;
  Rotation2d desiredHeading;
  boolean isWaypoint;
  boolean shouldNeverEnd;

/**
 * Command for moving to the target currently selected in the PhotonSubsystem. Without a desired heading, the robot turns so that the camera faces the target.
 * @param _targetOffset
 * the field oriented offset from the aprilTag to move towards
 * @param _isWaypoint
 * whether or not to use the larger tolerences meant for stop-and-go waypoints
 */
  public PhotonMoveToTarget(Translation2d _targetOffset, boolean _isWaypoint, boolean neverEnd) {
    addRequirements(SwerveSubsystem.getInstance());
    addRequirements(PhotonSubsystem.getInstance());
    targetOffset = _targetOffset;
    centerTarget=true;
    isWaypoint=_isWaypoint;
    shouldNeverEnd = neverEnd;
  }

/**
 * Command for moving to the target currently selected in the PhotonSubsystem
 * @param _targetOffset
 * the field oriented offset from the aprilTag to move towards
 * @param _desiredHeading
 * what direction the robot should be facing compared to the field
 * @param _isWaypoint
 * whether or not to use the larger tolerences meant for stop-and-go waypoints
 */
  public PhotonMoveToTarget(Translation2d _targetOffset, Rotation2d _desiredHeading, boolean _isWaypoint, boolean neverEnd) {
    addRequirements(SwerveSubsystem.getInstance());
    addRequirements(PhotonSubsystem.getInstance());
    targetOffset = _targetOffset;
    desiredHeading = _desiredHeading;
    centerTarget=false;
    isWaypoint=_isWaypoint;
    shouldNeverEnd = neverEnd;
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
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shouldNeverEnd) {
      return false;
    }
    
    if (isWaypoint){
      return SwerveSubsystem.getInstance().isAtPose(PhotonConfig.WAYPOINT_POS_TOLERANCE, PhotonConfig.WAYPOINT_ANGLE_TOLERANCE);
    } else {
      return SwerveSubsystem.getInstance().isAtPose(PhotonConfig.POS_TOLERANCE, PhotonConfig.ANGLE_TOLERANCE) 
          && !SwerveSubsystem.getInstance().isChassisMoving(PhotonConfig.VEL_TOLERANCE);
    }
  }
}
