package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.AdvantageUtil;
import frc.lib.lib2706.PoseBuffer;
import frc.robot.Config;
import frc.robot.Config.PhotonConfig;

public class SwerveSubsystem extends SubsystemBase {
  private final PigeonIMU gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  String tableName = "SwerveChassis";
  private NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable(tableName);
  private DoublePublisher pubCurrentAngle = swerveTable.getDoubleTopic("Current angle (deg)").publish(PubSubOption.periodic(0.02));
  private DoublePublisher pubCurrentPositionX = swerveTable.getDoubleTopic("Current positionX (m) ").publish(PubSubOption.periodic(0.02));
  private DoublePublisher pubCurrentPositionY = swerveTable.getDoubleTopic("Current positionY (m) ").publish(PubSubOption.periodic(0.02));
  private DoubleArrayPublisher pubCurrentPose = swerveTable.getDoubleArrayTopic("Pose ").publish(PubSubOption.periodic(0.02));

  // ProfiledPIDControllers for the pid control
  ProfiledPIDController pidControlX;
  double currentX;
  double desiredX;
  ProfiledPIDController pidControlY;
  double currentY;
  double desiredY;
  ProfiledPIDController pidControlRotation;
  double currentRotation;
  double desiredRotation;
  int tempSynchCounter = 0;

  /**
   * Counter to synchronize the modules relative encoder with absolute encoder when not moving.
   */
  private int moduleSynchronizationCounter = 0;
  
  private Field2d field;

  private PoseBuffer poseBuffer;

  private static SwerveSubsystem instance;
  public static SwerveSubsystem getInstance(){
      if(instance == null){
          instance = new SwerveSubsystem();
      }
      return instance;
  }

  private SwerveSubsystem() {
    gyro = new PigeonIMU(Config.Swerve.pigeonID);
    gyro.configFactoryDefault();
    // zeroGyro();

    

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Config.Swerve.Mod0.constants,"FL"),
          new SwerveModule(1, Config.Swerve.Mod1.constants,"FR"),
          new SwerveModule(2, Config.Swerve.Mod2.constants,"BL"),
          new SwerveModule(3, Config.Swerve.Mod3.constants,"BR")
        };

    swerveOdometry = new SwerveDriveOdometry(Config.Swerve.swerveKinematics, getYaw(), getPositions(), new Pose2d() );

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants kP = 5.0
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            3.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

    poseBuffer = new PoseBuffer();

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    pidControlX = new ProfiledPIDController(3, 0.0, 0.2,
            new TrapezoidProfile.Constraints(2,2));
    pidControlY = new ProfiledPIDController(3, 0.0, 0.2,
            new TrapezoidProfile.Constraints(2, 2));
    pidControlRotation = new ProfiledPIDController(2.0, 0, 0.4,
            new TrapezoidProfile.Constraints(8 * Math.PI, 8 * Math.PI));
            pidControlRotation.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void drive(
      ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
    Config.Swerve.swerveKinematics.toSwerveModuleStates(
      ChassisSpeeds.discretize(
        fieldRelative ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading()) :
            speeds, 0.02
      )
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Config.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
    setModuleStates(desiredStates, isOpenLoop, false);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean isDisableAntiJitter) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Config.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop, isDisableAntiJitter);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  private Rotation2d getYaw() {
    return (Config.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }
  
  public Command setHeadingCommand(Rotation2d rotation2d) {
    return Commands.runOnce(() -> resetOdometry(new Pose2d(getPose().getTranslation(), rotation2d)));
  }
  public Command setOdometryCommand(Pose2d pose) {
    return Commands.runOnce(() -> resetOdometry(pose));
  }
  public Command setLockWheelsInXCommand() {
    return run(() -> setModuleStates(
      new SwerveModuleState[]{
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
      }, true, true)
    );
  }
  public Command getDriveToPoseCommand(Pose2d desiredPose) {
    return runOnce(() -> resetDriveToPose()).andThen(run(() -> driveToPose(desiredPose)));
  }

  // Swerve actual driving methods
  public void resetDriveToPose() {
    pidControlX.reset(getPose().getX(),getFieldRelativeSpeeds().vxMetersPerSecond);
    pidControlY.reset(getPose().getY(),getFieldRelativeSpeeds().vyMetersPerSecond);
    pidControlRotation.reset(getPose().getRotation().getRadians(),getFieldRelativeSpeeds().omegaRadiansPerSecond);
  }

  public void driveToPose(Pose2d pose) {
    //update the currentX and currentY
    
    currentX = getPose().getX();
    currentY = getPose().getY();
    currentRotation = getPose().getRotation().getRadians();

    desiredX = pose.getX();
    desiredY = pose.getY();
    desiredRotation = pose.getRotation().getRadians();

    double x = pidControlX.calculate(currentX, desiredX);
    double y = pidControlY.calculate(currentY, desiredY);
    double rot = 0;
    if (Math.abs(currentRotation - desiredRotation) > PhotonConfig.ANGLE_TOLERANCE) {
      rot = pidControlRotation.calculate(currentRotation, desiredRotation);
    }

    drive(new ChassisSpeeds(x, y, rot), true, false);
  }

  public boolean isAtPose(double tol, double angleTol) {
    return Math.abs(currentX - desiredX) < tol && Math.abs(currentY - desiredY) < tol
            && Math.abs(currentRotation - desiredRotation) < angleTol;
  }

  /**
   * Get a pose at the given timestamp. 
   * Returns an empty Optional if the buffer is empty or doesn't go back far enough.
   * 
   * @param timestampSeconds The timestamp for the pose to get, matching WPILib PoseEstimator's 
   *                         timestamps (which matches PhotonVision and Limelight)
   * @return An Optional of the Pose2d or an empty Optional.
   */
  public Optional<Pose2d> getPoseAtTimestamp(double timestampSeconds) {
    return(poseBuffer.getPoseAtTimestamp(timestampSeconds));
  }

  @Override
  public void periodic() {
    for (SwerveModule mod : mSwerveMods) {
        mod.periodic();
    }
    SwerveModulePosition[] tempGetPositions = getPositions();
    SwerveModuleState[] tempGetStates = getStates();
    swerveOdometry.update(getYaw(), tempGetPositions);
    field.setRobotPose(getPose());

  
    // If the robot isn't moving synchronize the encoders every 100ms (Inspired by democrat's SDS
    // lib)
    // To ensure that everytime we initialize it works.
    if (!isChassisMoving()) {
      if (++moduleSynchronizationCounter > 6 && isSwerveNotSynched()) {
        synchSwerve();
        System.out.println("Resynced" + ++tempSynchCounter);
        moduleSynchronizationCounter = 0;
      }
    }
    else {
      moduleSynchronizationCounter = 0;
    }

    poseBuffer.addPoseToBuffer(getPose());

    pubCurrentAngle.accept(getPose().getRotation().getDegrees());
    pubCurrentPositionX.accept(getPose().getX());
    pubCurrentPositionY.accept(getPose().getY());
    pubCurrentPose.accept(AdvantageUtil.deconstruct(getPose()));


  }
  
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = Config.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates, false);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Config.Swerve.swerveKinematics.toChassisSpeeds(getStates());
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  public ChassisSpeeds getFieldRelativeSpeeds()
  {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getHeading());
  }

  public boolean isChassisMoving()
  {
    double sumVelocity = 0;
    for (SwerveModule mod : mSwerveMods) {
      sumVelocity += Math.abs(mod.getState().speedMetersPerSecond);
    }

    if (sumVelocity <= .01) {
      return false;
    }

    else
    {
      return true;
    }

  }
  public boolean isSwerveNotSynched() {
    for (SwerveModule module : mSwerveMods) {
      if (!module.isModuleSynced()) {
        return(true);
      }
    }
    return(false);
  }
  public void synchSwerve() {
    for (SwerveModule module : mSwerveMods) {
      module.resetToAbsolute();
    }
  }
}
