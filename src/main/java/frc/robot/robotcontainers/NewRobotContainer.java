  
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.lib2706.TunableNumber;
import frc.lib.lib2706.XBoxControllerUtil;
import frc.robot.Config;
import frc.robot.Config.ArmSetPoints;
import frc.robot.Config.PhotonConfig;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.Config.Swerve.TeleopSpeeds;
import frc.robot.Robot;
import frc.robot.commands.BlingCommand;
import frc.robot.commands.BlingCommand.BlingColour;
import frc.robot.commands.ClimberRPM;
import frc.robot.commands.CombinedCommands;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.RotateAngleToVisionSupplier;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.RumbleJoystick;
import frc.robot.commands.SetArm;
import frc.robot.commands.SubwooferShot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.AutoSelector;
import frc.robot.subsystems.IntakeStateMachine.IntakeModes;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterStateMachine.ShooterModes;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class NewRobotContainer extends RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final CommandXboxController testJoystick = new CommandXboxController(2);

  /* Create Subsystems in a specific order */
  private final SwerveSubsystem s_Swerve = SwerveSubsystem.getInstance();
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

  /* Auto */
  private AutoRoutines m_autoRoutines;
  private AutoSelector m_autoSelector;

  /* Default Command */
  private Command m_swerveDefaultCommand;

  private TunableNumber shooterTargetRPM = new TunableNumber("Shooter/Target RPM", 0);
  private TunableNumber shooterDesiredVoltage = new TunableNumber("Shooter/desired Voltage", 0);
  private TunableNumber armAngleDeg = new TunableNumber("Arm/ArmTuning/setAngleDeg", 5.0);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public NewRobotContainer() {
    /*  Setup default commands */
    m_swerveDefaultCommand = new TeleopSwerve(driver);
    s_Swerve.setDefaultCommand(m_swerveDefaultCommand);
    if (!Config.disableStateBasedProgramming) {
      intake.setDefaultCommand(intake.defaultIntakeCommand());
      shooter.setDefaultCommand(shooter.defaultShooterCommand(()-> intake.isNoteIn()));
    } else {
      // shooter.setDefaultCommand(new Shooter_PID_Tuner(() -> 0));
    }

    configureButtonBindings();

     // Setup auto
    m_autoRoutines = new AutoRoutines();
    m_autoSelector = new AutoSelector();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() { 
    // Set bling to purple when note is in

    new Trigger(() -> intake.isBackSensorActive()).onTrue(CombinedCommands.strobeToSolidBlingCommand())
                                                  .onFalse(new BlingCommand(BlingColour.DISABLED));

    new Trigger(() -> intake.isBackSensorLongActive() && DriverStation.isTeleopEnabled()).onTrue(Commands.parallel(
            new RumbleJoystick(driver, RumbleType.kBothRumble, 0.75, 0.4, false),
            new RumbleJoystick(operator, RumbleType.kBothRumble, 0.75, 0.4, false))
    );          

    /**
     * Driver Controls
     * KingstonV1: https://drive.google.com/file/d/1gDgxnz-agWGoYmTTRfViVPwR7O2H80mh
     */
    // Core Swerve Buttons
    driver.back().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d(0)));
    driver.leftBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW)))
                       .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));

    driver.rightBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setFieldRelative(false)))
                       .onFalse(Commands.runOnce(() -> TeleopSwerve.setFieldRelative(true)));

    driver.start().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().synchSwerve()));

    // Commands that take control of the rotation stick
    driver.y().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(0)));
    driver.x().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(90)));
    driver.a().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(180)));
    driver.b().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(270)));   
    driver.rightTrigger().whileTrue(new RotateAngleToVisionSupplier(driver, "photonvision/" + PhotonConfig.frontCameraName));
    
    // Vision scoring commands with no intake, shooter, arm
    // driver.leftTrigger().whileTrue(new SelectByAllianceCommand( // Implement command group that also controls the arm, intake, shooter
    //   PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.AMP_BLUE, driver), 
    //   PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.AMP_RED, driver)));

    // driver.rightTrigger().whileTrue(new SelectByAllianceCommand( // Implement command group that also controls the arm, intake, shooter
    //   PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.RIGHT_SPEAKER_BLUE, driver), 
    //   PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.LEFT_SPEAKER_RED, driver)));

    driver.leftTrigger().whileTrue(CombinedCommands.centerSpeakerVisionShot(driver, PhotonPositions.FAR_SPEAKER_BLUE, PhotonPositions.FAR_SPEAKER_RED))
            .onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW)))
            .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));

    // driver.leftTrigger().whileTrue(CombinedCommands.podiumSourceSideSpeakerVisionShot(driver, PhotonPositions.PODIUM_SOURCESIDE_BLUE, PhotonPositions.PODIUM_SOURCESIDE_RED))
    //         .onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW)))
    //         .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));

    // Complete vision scoring commands with all subsystems
    // if (Config.disableStateBasedProgramming) {
    //   // Score in amp with vision using simple intake/shooter
    //   driver.leftBumper().whileTrue(CombinedCommands.simpleAmpScoreWithVision(driver));

    //   // Score in speaker with vision using simple intake/shooter
    //   driver.rightBumper().whileTrue(CombinedCommands.simpleSpeakerScoreWithVision(driver, ArmSetPoints.SPEAKER_KICKBOT_SHOT, PhotonPositions.RIGHT_SPEAKER_BLUE, PhotonPositions.RIGHT_SPEAKER_RED));

    // } else {
    //   // Score in amp with vision using stateful intake/shooter
    //   driver.leftBumper().whileTrue(CombinedCommands.statefulAmpScoreWithVision(driver));

    //   // Score in speaker with vision using stateful intake/shooter
    //   driver.leftBumper().whileTrue(CombinedCommands.statefulSpeakerScoreWithVision(driver, ArmSetPoints.SPEAKER_KICKBOT_SHOT, PhotonPositions.RIGHT_SPEAKER_BLUE, PhotonPositions.RIGHT_SPEAKER_RED));
    // }

    /**
     * Operator Controls
     * KingstonV1: https://drive.google.com/file/d/18HyIpIeW08CC6r6u-Z74xBWRv9opHnoZ
     */
    // Arm
    operator.y().onTrue(new SetArm(()->ArmSetPoints.AMP.angleDeg)); // Amp
    operator.b().onTrue(new SetArm(()->ArmSetPoints.IDLE.angleDeg)); // Idle
    operator.a().onTrue(new SetArm(()->ArmSetPoints.NO_INTAKE.angleDeg)); // Pickup
    operator.x().onTrue(new SetArm(()->ArmSetPoints.SPEAKER_KICKBOT_SHOT.angleDeg));
    // Climber
    operator.leftTrigger(0.10).and(operator.back()).whileTrue(new ClimberRPM(()-> MathUtil.applyDeadband(operator.getLeftTriggerAxis(), 0.35) * 0.5));

    // Eject the note from the front with start
    operator.start()
      .whileTrue(Commands.run(() -> intake.setVoltage(-12), intake))
      .onFalse(Commands.runOnce(() -> intake.stop()));
  
    
    // Simple shooter and intake
    if (Config.disableStateBasedProgramming) {
      intake.setStateMachineOff();
      shooter.setStateMachineOff();

      // Intake note with leftTrigger
    
      //operator.leftTrigger(0.3).whileTrue(
      operator.leftBumper()
      .whileTrue(CombinedCommands.armIntake())
      .onFalse(new SetArm(()->ArmSetPoints.NO_INTAKE.angleDeg))
      .onFalse(new MakeIntakeMotorSpin(9.0,0).withTimeout(1).until(() -> intake.isBackSensorActive()));


      //NOTE: right Trigger has been assigned to climber
      operator.rightTrigger(0.3).whileTrue(CombinedCommands.simpleShootNoteAmp());
      // Shoot note with leftBumper
      // operator.rightBumper().whileTrue(CombinedCommands.simpleShootNoteSpeaker(1))
      //                       .onTrue(new SetArm(()->ArmSetPoints.SPEAKER_KICKBOT_SHOT.angleDeg));

      operator.rightBumper().onTrue(new SubwooferShot(
        operator.rightBumper(), 
        ArmSetPoints.SPEAKER_KICKBOT_SHOT.angleDeg, 
        2820, 
        2700));


    // State based shooter and intake
    } else {
      intake.setStateMachineOn();
      shooter.setStateMachineOn();

      // Intake note with leftTrigger
      operator.leftTrigger(0.3) //Intake the Note
          .whileTrue(Commands.runOnce(()-> intake.setMode(IntakeModes.INTAKE)))
          .whileFalse(Commands.runOnce(()->intake.setMode(IntakeModes.STOP_INTAKE)));   

      // Toggle to spin up or spin down the shooter with rightBumper
      operator.rightBumper().onTrue(shooter.toggleSpinUpCommand(ShooterModes.SHOOT_SPEAKER));  
          
      // Shoot note with leftBumper
      // operator.leftBumper().onTrue(CombinedCommands.statefulShootNote());

      // Eject the note from the front with leftPOV
      XBoxControllerUtil.leftPOV(operator).debounce(0.1)
        .whileTrue(Commands.runOnce(()-> intake.setMode(IntakeModes.RELEASE)))
        .whileFalse(Commands.runOnce(()->intake.setMode(IntakeModes.STOP_INTAKE))); 
    }


    /**
     * Testing button bindings
     */
    // SwerveModuleState[] moduleStatesForwards = {
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
    // };
    // testJoystick.a().whileTrue(Commands.run(
    //   () -> SwerveSubsystem.getInstance().setModuleStates(moduleStatesForwards, true, true)
    // ));

    // SwerveModuleState[] moduleStatesSideways = {
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    // };
    // testJoystick.a().whileTrue(Commands.run(
    //   () -> SwerveSubsystem.getInstance().setModuleStates(moduleStatesSideways, true, true)
    // ));
    // Let testJoystick control swerve. Note disables driver joystick swerve. Never commit this line.
    // s_Swerve.setDefaultCommand(new TeleopSwerve(testJoystick));
    // testJoystick.back().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d()));
    // testJoystick.b().onTrue(SwerveSubsystem.getInstance().setOdometryCommand(new Pose2d(3,3,new Rotation2d(0))));
    // testJoystick.a().whileTrue(PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.FAR_SPEAKER_RED, driver))
    //           .onFalse(Commands.runOnce(()->{},SwerveSubsystem.getInstance()));

    // testJoystick.x() //Drives the note into the shooter
    //   .whileTrue(Commands.runOnce(()-> intake.setMode(shooter.isReadyToShoot() ? IntakeModes.SHOOT : IntakeModes.STOP_INTAKE)))
    //   .whileFalse(Commands.runOnce(()->intake.setMode(IntakeModes.STOP_INTAKE)));  
    
    // testJoystick.leftBumper().whileTrue(
    //       new MakeIntakeMotorSpin(9.0,0));
    
    // testJoystick.start().onTrue( new SetArm(armAngleDeg));
    // testJoystick.back().whileTrue(new Shooter_PID_Tuner(shooterTargetRPM));
    // testJoystick.rightBumper().whileTrue(CombinedCommands.simpleShootNoteSpeaker(1, () -> shooterTargetRPM.getAsDouble(), 50)); 

    // testJoystick.rightTrigger().whileTrue(new Shooter_PID_Tuner(()->shooterTargetRPM.getAsDouble()));
 
    // testJoystick.a().onTrue(PhotonSubsystem.getInstance().getResetCommand(4));
    // testJoystick.b().onTrue(SwerveSubsystem.getInstance().setOdometryCommand(new Pose2d(3, 3, Rotation2d.fromDegrees(0))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *leop
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    int autoId = m_autoSelector.getAutoId();
    System.out.println("*********************** Auto Id"+autoId);

    return m_autoRoutines.getAutonomousCommand(autoId);
  }
}
