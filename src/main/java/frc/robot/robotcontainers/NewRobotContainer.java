  
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;


import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.Config.Swerve.TeleopSpeeds;
import frc.robot.Robot;
import frc.robot.commands.ArmFFTestCommand;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.RotateAngleToVision;
import frc.robot.commands.Shooter_tuner;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.AutoSelector;
import frc.robot.subsystems.ArmPneumaticsSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
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

  private final SwerveSubsystem s_Swerve = SwerveSubsystem.getInstance();
  //private final SwerveSubsystem s_Swerve = SwerveSubsystem.getInstance();

  String tableName = "SwerveChassis";
  private NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable(tableName);
  private IntegerEntry entryAutoRoutine;

  AutoSelector m_autoSelector;

  /* Create Subsystems in a specific order */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public NewRobotContainer() {
    AutoRoutines.registerCommandsToPathplanner();

    /*  Setup default commands */
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            driver
        )
    );

    entryAutoRoutine = swerveTable.getIntegerTopic("Auto Selector ID").getEntry(0);
    entryAutoRoutine.setDefault(0);

    // Configure the button bindings
    configureButtonBindings();

    m_autoSelector = new AutoSelector();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() { 
    
    
    /* Driver Controls */  
    driver.start().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d(0)));
  

    // driver.back().whileTrue(new AutoRoutines().getAutonomousCommand(2));
    driver.leftTrigger().whileTrue(new PathPlannerAuto("tuneAutoX"));
    driver.rightTrigger().whileTrue(new PathPlannerAuto("tuneAutoY"));
    
    driver.leftBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW))).onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));
    
    driver.y().whileTrue(new RotateAngleToVision(s_Swerve, driver, 0));
//     driver.b().whileTrue(new RotateAngleToVision(s_Swerve, driver, Math.PI / 2.0));
//     driver.a().whileTrue(new RotateAngleToVision(s_Swerve, driver, Math.PI));
    driver.x().whileTrue(new RotateAngleToVision(s_Swerve, driver, -Math.PI / 2.0));


    driver.back().whileTrue(SwerveSubsystem.getInstance().setLockWheelsInXCommand());
    driver.b().onTrue(SwerveSubsystem.getInstance().setOdometryCommand(new Pose2d(3,3,new Rotation2d(0))));
    driver.a().whileTrue(PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.FAR_SPEAKER_RED)).onFalse(Commands.runOnce(()->{},SwerveSubsystem.getInstance()));

    /* Operator Controls */
    operator.a().whileTrue(new MakeIntakeMotorSpin(0.6, 0));
    operator.x().whileTrue(new Shooter_tuner(12));

    //operator.y().whileTrue (new ArmFFTestCommand(operator, 3, true));
  
    operator.b().whileTrue(new IntakeControl(true));
    operator.y().whileTrue(new IntakeControl(false));
    operator.start().whileTrue(Commands.deadline(
      Commands.sequence(
        new IntakeControl(false), 
        new WaitCommand(0.5), 
        new IntakeControl(true).withTimeout(2)),
      new Shooter_tuner(12)
    ));
    //turns brakes off
    operator.rightBumper().onTrue(Commands.runOnce(() -> ArmPneumaticsSubsystem.getInstance().controlBrake(false, true)));

    //turns brakes on
    operator.rightTrigger().onTrue(Commands.runOnce(() -> ArmPneumaticsSubsystem.getInstance().controlBrake(true, true)));
  }
  
  public Command autoResetOdometryCommand() {
    Rotation2d prevHeading = s_Swerve.getHeading();
    return(new SequentialCommandGroup(getAutonomousCommand(), s_Swerve.setHeadingCommand(prevHeading)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // int autoId = m_autoSelector.getAutoId();
    // System.out.println("*********************** Auto Id"+autoId);
    //  return new AutoRoutines().getAutonomousCommand(autoId);
    int autoRoutineID = (int)entryAutoRoutine.get();
    System.out.println(autoRoutineID);
    return new AutoRoutines().getAutonomousCommand(autoRoutineID);
  }
}
