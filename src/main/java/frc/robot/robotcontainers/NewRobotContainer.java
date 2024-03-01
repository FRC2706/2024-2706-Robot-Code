  
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.lib2706.SelectByAllianceCommand;
import frc.lib.lib2706.TunableNumber;
import frc.lib.lib2706.XBoxControllerUtil;
import frc.robot.Config;
import frc.robot.Config.PhotonConfig;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.Config.Swerve.TeleopSpeeds;
import frc.robot.Robot;
import frc.robot.commands.CombinedCommands;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.RotateAngleToVisionSupplier;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.SetArm;
import frc.robot.commands.Shooter_Voltage;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.AutoSelector;
import frc.robot.subsystems.IntakeStatesMachine.IntakeModes;
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

  private TunableNumber shooterTargetRPM = new TunableNumber("Shooter/Target RPM", 0);
  private TunableNumber shooterDesiredVoltage = new TunableNumber("Shooter/desired Voltage", 0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public NewRobotContainer() {
    // Setup auto
    m_autoRoutines = new AutoRoutines();
    m_autoSelector = new AutoSelector();

    /*  Setup default commands */
    s_Swerve.setDefaultCommand(new TeleopSwerve(driver));
    intake.setDefaultCommand(intake.defaultIntakeCommand());
    shooter.setDefaultCommand(shooter.defaultShooterCommand(()-> intake.isNoteIn()));

    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() { 
    /**
     * Driver Controls
     * KingstonV1: https://drive.google.com/file/d/1gDgxnz-agWGoYmTTRfViVPwR7O2H80mh
     */
    // Core Swerve Buttons
    driver.back().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d(0)));
    driver.leftBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW)))
                       .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));
    // driver.rightBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setFieldOriented())) // setFieldOriented is not yet implemeneted
    //                    .onFalse(Commands.runOnce(() -> TeleopSwerve.setCameraOriented())); // setCameraOriented is not yet implemeneted

    // Commands that take control of the rotation stick
    driver.y().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(0)));
    driver.x().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(90)));
    driver.a().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(180)));
    driver.b().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(270)));

    driver.start().whileTrue(new RotateAngleToVisionSupplier(driver, "photonvision/" + PhotonConfig.apriltagCameraName));    

    // Vision scoring commands with no intake, shooter, arm
    driver.leftBumper().whileTrue(new SelectByAllianceCommand( // Implement command group that also controls the arm, intake, shooter
      PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.AMP_BLUE, driver), 
      PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.AMP_RED, driver)));

    driver.rightBumper().whileTrue(new SelectByAllianceCommand( // Implement command group that also controls the arm, intake, shooter
      PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.MIDDLE_SPEAKER_BLUE, driver), 
      PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.MIDDLE_SPEAKER_RED, driver)));

    /**
     * Operator Controls
     * KingstonV1: https://drive.google.com/file/d/1Aso1cqR4icCaf3xc73Lvr470EWszzKEA
     */
    // Arm
    operator.y().onTrue(new SetArm(0)); // Amp
    operator.b().onTrue(new SetArm(0)); // Idle
    operator.a().onTrue(new SetArm(0)); // Pickup
    XBoxControllerUtil.leftPOV(operator).debounce(0.1).onTrue(new SetArm(0)); // Kickbot Shot

    // Climber
    // operator.leftTrigger() // Implement climb command here

    // Simple shooter and intake
    if (Config.disableStateBasedProgramming) {
      intake.setStateMachineOff();
      shooter.setStateMachineOff();

      // Intake note with leftTrigger
      operator.leftTrigger(0.3).whileTrue(
        Commands.run(() -> intake.setVoltage(8), intake));

      // Toggle to spin up or spin down the shooter with rightBumper
      operator.rightBumper().toggleOnTrue(new Shooter_Voltage(()->8));

      // Shoot note with leftBumper
      operator.leftBumper().whileTrue(CombinedCommands.simpleShootNote());

      // Eject the note from the front with leftPOV
      XBoxControllerUtil.leftPOV(operator).debounce(0.1).whileTrue(
        Commands.run(() -> intake.setVoltage(-6), intake));

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
      operator.leftBumper().onTrue(CombinedCommands.statefulShootNote());

      // Eject the note from the front with leftPOV
      XBoxControllerUtil.leftPOV(operator).debounce(0.1)
        .whileTrue(Commands.runOnce(()-> intake.setMode(IntakeModes.RELEASE)))
        .whileFalse(Commands.runOnce(()->intake.setMode(IntakeModes.STOP_INTAKE))); 
    }


    /**
     * Testing button bindings
     */
    // Let testJoystick control swerve. Note disables driver joystick swerve. Never commit this line.
    // s_Swerve.setDefaultCommand(new TeleopSwerve(testJoystick));

    // testJoystick.b().onTrue(SwerveSubsystem.getInstance().setOdometryCommand(new Pose2d(3,3,new Rotation2d(0))));
    // testJoystick.a().whileTrue(PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.FAR_SPEAKER_RED, driver))
    //           .onFalse(Commands.runOnce(()->{},SwerveSubsystem.getInstance()));

    // testJoystick.x() //Drives the note into the shooter
    //   .whileTrue(Commands.runOnce(()-> intake.setMode(shooter.isReadyToShoot() ? IntakeModes.SHOOT : IntakeModes.STOP_INTAKE)))
    //   .whileFalse(Commands.runOnce(()->intake.setMode(IntakeModes.STOP_INTAKE)));    
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
