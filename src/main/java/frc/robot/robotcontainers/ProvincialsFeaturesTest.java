// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.AutoSelector;
import frc.robot.subsystems.IntakeStatesMachine.IntakeModes;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterStateMachine.ShooterModes;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class ProvincialsFeaturesTest extends RobotContainer {
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

  private TunableNumber shooterTargetRPM = new TunableNumber("Shooter/Target RPM", 0);
  private TunableNumber shooterDesiredVoltage = new TunableNumber("Shooter/desired Voltage", 0);
  private TunableNumber armAngleDeg = new TunableNumber("Arm/ArmTuning/setAngleDeg", 5.0);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public ProvincialsFeaturesTest() {
    /*  Setup default commands */
    s_Swerve.setDefaultCommand(new TeleopSwerve(driver));
    intake.setDefaultCommand(intake.defaultIntakeCommand());
    shooter.setDefaultCommand(shooter.defaultShooterCommand(()-> intake.isNoteIn(), ()->0));

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
    
    driver.leftTrigger().whileTrue(CombinedCommands.centerSpeakerVisionShot(driver, PhotonPositions.FAR_SPEAKER_BLUE, PhotonPositions.FAR_SPEAKER_RED))
            .onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW)))
            .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));


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
      operator.rightBumper().whileTrue(CombinedCommands.simpleShootNoteSpeaker(1))
                            .onTrue(new SetArm(()->ArmSetPoints.SPEAKER_KICKBOT_SHOT.angleDeg));

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
