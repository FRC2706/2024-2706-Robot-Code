  
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
import frc.robot.Config.ArmSetPoints;
import frc.robot.Config.PhotonConfig;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.Config.Swerve.TeleopSpeeds;
import frc.robot.Robot;
import frc.robot.commands.ClimberRPM;
import frc.robot.commands.CombinedCommands;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.RotateAngleToVisionSupplier;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.SetArm;
import frc.robot.commands.Shooter_PID_Tuner;
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
public class ContainerForTest extends RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final CommandXboxController testJoystick = new CommandXboxController(2);//More buttons

  /* Create Subsystems in a specific order */
  private final SwerveSubsystem s_Swerve = SwerveSubsystem.getInstance();
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

  private TunableNumber shooterTargetRPM = new TunableNumber("Shooter/Target RPM", 0);
  private TunableNumber shooterDesiredVoltage = new TunableNumber("Shooter/desired Voltage", 0);
  private TunableNumber intakeDesiredVoltage = new TunableNumber("Intake/desired Voltage", 0);
  private TunableNumber armDesiredAngle = new TunableNumber("Arm/desired Angle", 0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public ContainerForTest() {
    
    /*  Setup default commands */
    s_Swerve.setDefaultCommand(new TeleopSwerve(driver));

    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() { 

    /*-------------------------- Driver Controls --------------------------*/

    // Core Swerve Buttons
    driver.back().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d(0)));
    driver.leftBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW)))
                       .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));
    
    // Target a Cardinal Point
    driver.y().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(0)));
    driver.x().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(90)));
    driver.a().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(180)));
    driver.b().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(270)));

    driver.start().whileTrue(new RotateAngleToVisionSupplier(driver, "photonvision/" + PhotonConfig.apriltagCameraName));    

    /*-------------------------- Operator Controls --------------------------*/

    operator.y()// Arm
      .whileTrue(new SetArm(()->armDesiredAngle.get()));
    
    operator.rightTrigger(0.25)// Climber
      .whileTrue(new ClimberRPM(()->  driver.getRightTriggerAxis()));

    operator.a()// Intake note with leftTrigger
      .whileTrue(Commands.run(() -> intake.setVoltage(intakeDesiredVoltage.get()), intake))
      .whileFalse(Commands.run(() -> intake.setVoltage(0), intake));

    operator.x()// Toggle to spin up or spin down the shooter with rightBumper
      .whileTrue(new Shooter_Voltage(()->shooterDesiredVoltage.get()));

    operator.leftBumper()// Shoot note with leftBumper
      .onTrue(CombinedCommands.simpleShootNoteSpeaker());

    // Eject the note from the front with leftPOV
    XBoxControllerUtil.leftPOV(operator).debounce(0.1)
      .whileTrue(Commands.run(() -> intake.setVoltage(intakeDesiredVoltage.get()), intake))
      .whileFalse(Commands.run(() -> intake.setVoltage(0), intake));
    
    /*-------------------------- More Buttons --------------------------*/
    testJoystick.a().whileTrue(new Shooter_PID_Tuner(()-> shooterTargetRPM.get()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *leop
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
