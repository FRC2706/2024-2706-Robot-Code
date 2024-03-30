  
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
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.lib2706.SelectByAllianceCommand;
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
import frc.robot.commands.IntakeControl;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.RotateAngleToVisionSupplier;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.RumbleJoystick;
import frc.robot.commands.SetArm;
import frc.robot.commands.Shooter_PID_Tuner;
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
public class ContainerForTesting extends RobotContainer {
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
  public ContainerForTesting() {
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
   
    /**
     * Operator Controls
     * KingstonV1: https://drive.google.com/file/d/18HyIpIeW08CC6r6u-Z74xBWRv9opHnoZ
     */
    // Arm
      operator.y().onTrue(new SetArm(()->armAngleDeg.get())).onFalse(new SetArm(()->-0.1)); // Amp
      operator.x().whileTrue(new Shooter_PID_Tuner(()->4000))//()->shooterTargetRPM.get()))
                  .onFalse(Commands.run(()->shooter.setVoltage(0)));
      operator.a().whileTrue(Commands.run(()->shooter.setVoltage(12)))
                  .onFalse(Commands.run(()->shooter.setVoltage(0)));
      operator.rightBumper().onTrue(new MakeIntakeMotorSpin(9.0, 0)).onFalse(new MakeIntakeMotorSpin(0.0, 0));
      operator.leftBumper().onTrue(new MakeIntakeMotorSpin(-4.0, 0)).onFalse(new MakeIntakeMotorSpin(0.0, 0));
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
