// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//imports
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config;
import frc.robot.Config.PhotonConfig;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.commands.PhotonMoveToTarget;
import frc.robot.commands.RumbleJoystick;

//class
public class PhotonSubsystem extends SubsystemBase {

  //constants
  
  //declarations
  private static PhotonSubsystem instance;
  private DoubleArrayPublisher pubSetPoint;
  private DoublePublisher pubRange, pubYaw;
  private PhotonCamera camera1;
  private Translation2d targetPos;
  private Rotation2d targetRotation;
  private LinearFilter filteryaw = LinearFilter.movingAverage(PhotonConfig.maxNumSamples);
  private LinearFilter filterX = LinearFilter.movingAverage(PhotonConfig.maxNumSamples);
  private LinearFilter filterY = LinearFilter.movingAverage(PhotonConfig.maxNumSamples);
  private int numSamples;
  private int id;

  

  public static PhotonSubsystem getInstance(){
    if (instance == null){
      instance = new PhotonSubsystem();
    }
    return instance;
  }

  /** Creates a new photonAprilTag. */
  private PhotonSubsystem() {
    //name of camera, change if using multiple cameras
    camera1 = new PhotonCamera(PhotonConfig.apriltagCameraName);
    //networktable publishers
    pubSetPoint = NetworkTableInstance.getDefault().getTable(PhotonConfig.networkTableName).getDoubleArrayTopic("PhotonAprilPoint").publish(PubSubOption.periodic(0.02));
    pubRange = NetworkTableInstance.getDefault().getTable(PhotonConfig.networkTableName).getDoubleTopic("Range").publish(PubSubOption.periodic(0.02));
    pubYaw = NetworkTableInstance.getDefault().getTable(PhotonConfig.networkTableName).getDoubleTopic("Yaw").publish(PubSubOption.periodic(0.02));
    reset(-1);
  }

  public void reset(int desiredId) {
    filterX.reset();
    filterY.reset();
    filteryaw.reset();
    //set target info to the robot's info
    targetRotation = SwerveSubsystem.getInstance().getPose().getRotation();
    targetPos = SwerveSubsystem.getInstance().getPose().getTranslation();
    //initialize vars
    numSamples = 0;
    id = desiredId;
  }

  /**
   * Command for removing photon data and changing the target id to look for.
   * @param tagid
   * the id that the subsystem will now look for
   * @return
   * the command
   */
  public Command getResetCommand(int tagid){
    return runOnce(() -> reset(tagid));
  }
  
  //publishes yaw
  public Rotation2d getYaw (){
    return targetRotation;
  }

  /**
   * Command that does nothing until photonSubsystem has enough data to track a target
   * @param tagid
   * resets the subsystem and its id
   * @return
   * the command
   */
  public Command getWaitForDataCommand(int tagid){
    return new FunctionalCommand(() -> reset(tagid), ()->{}, (interrupted) ->{}, ()->hasData(), this);
  }

/**
 * this is the command for going to a specific position based off of an apriltag during teleoperated,------------------------       
 * 
 * during Auto, please use the positions with PhotonMoveToTarget
 * @param spacePositions
 * a PhotonPositions object from the config file
 * @return
 * the command to run
 */
  public Command getAprilTagCommand(PhotonPositions spacePositions, CommandXboxController driverStick){
    Command moveToTargetCommands;
    if (spacePositions.hasWaypoint) {
      moveToTargetCommands = Commands.sequence(
        new PhotonMoveToTarget(spacePositions.waypoint,true),
        new PhotonMoveToTarget(spacePositions.destination, false)
      );
    } else {
      moveToTargetCommands = new PhotonMoveToTarget(spacePositions.destination, false);
    }

    return Commands.sequence(
      new ProxyCommand(getWaitForDataCommand(spacePositions.id)), // Proxy this command to prevent PhotonSubsystem requirement conflicting with PhotonMoveToTarget's requirements
      new ScheduleCommand(new RumbleJoystick(driverStick, RumbleType.kBothRumble, 0.5, 0.2, true)),
      new ProxyCommand(moveToTargetCommands) // Proxy this command to prevent SwerveSubsystem requirement conflicting with WaitForDataCommand's requirements
    );
  }

  public Translation2d getTargetPos(){
    return targetPos;
  }

  public Rotation2d getTargetRotation(){
    return targetRotation;
  }

  public boolean hasData() {
    //if data is the max that the filters hold
    return(numSamples >= PhotonConfig.maxNumSamples);
  }

  private double range(double y) {
    y = Math.toRadians(y);
    y += PhotonConfig.CAMERA_PITCH.getRadians();

    if (id-4 < 0) {
      return 0;
    }else if(id-4>= Config.PhotonConfig.APRIL_HEIGHTS.length){
      return 0;
    }

    return (Config.PhotonConfig.APRIL_HEIGHTS[id-4]-PhotonConfig.CAMERA_HEIGHT)/Math.tan(y);
  }

  private PhotonTrackedTarget biggestTarget(List<PhotonTrackedTarget> targets) {
    PhotonTrackedTarget biggestTarget=null;

    double tallest = 0;
    for (PhotonTrackedTarget t:targets) {
      List<TargetCorner> corners = t.getDetectedCorners();
      double sizeY = corners.get(1).y-corners.get(3).y;
      if (sizeY > tallest) {
        tallest = sizeY;
        biggestTarget = t;
      }
    }
    return (biggestTarget);
  }

  private Pose2d convertToField(double range, Rotation2d yaw, Pose2d odometryPose) {
    Rotation2d fieldOrientedTarget = yaw.rotateBy(odometryPose.getRotation());
    Translation2d visionXY = new Translation2d(range, yaw);
    Translation2d robotRotated = visionXY.rotateBy(PhotonConfig.cameraOffset.getRotation());
    Translation2d robotToTargetRELATIVE = robotRotated.plus(PhotonConfig.cameraOffset.getTranslation());
    Translation2d robotToTarget = robotToTargetRELATIVE.rotateBy(odometryPose.getRotation());
    return new Pose2d(robotToTarget.plus(odometryPose.getTranslation()), fieldOrientedTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera1.getLatestResult();
    if (result.hasTargets()){
      //get the swerve pose at the time that the result was gotten
      Optional<Pose2d> optPose= SwerveSubsystem.getInstance().getPoseAtTimestamp(result.getTimestampSeconds());
      //for security reasons
      if (optPose.isEmpty()){
        return;
      }
      Pose2d odometryPose = optPose.get();

      PhotonTrackedTarget target = null;
      //currently chooses lowest id if sees two april tags

      List<PhotonTrackedTarget> allTargets = result.getTargets();
      if (allTargets.size()==0){
        return;
      }
      if (id == -1){
        target = biggestTarget(allTargets);

        if (target == null) {
          return;
        }
        id = target.getFiducialId();
      } else{
        for (PhotonTrackedTarget t:allTargets){
          if (t.getFiducialId() == id){
           target = t;
           break;
          }
        }
        if (target == null){
          return;
        }
      }
      
      //get tag info
      //calculate yaw
      Rotation2d yaw = Rotation2d.fromDegrees(target.getYaw()*-1);
      //calculate range
      double range = (range(target.getPitch()));
      if (range < 6 && range > 1.5){
        double va = (range-1.5)/10;

        range*=(1-va);
      }
      //convert to field quordinates
      Pose2d fieldToTarget = convertToField(range, yaw, odometryPose);
      //update rolling averages
      targetPos = new Translation2d(filterX.calculate(fieldToTarget.getX()),filterY.calculate(fieldToTarget.getY()));
      targetRotation = Rotation2d.fromDegrees(filteryaw.calculate(fieldToTarget.getRotation().getDegrees()));
      numSamples ++;

      //publish to networktables
      pubSetPoint.accept(new double[]{targetPos.getX(),targetPos.getY(),targetRotation.getRadians()});
      pubRange.accept(range);
      pubYaw.accept(yaw.getDegrees());
    }
  }
}
