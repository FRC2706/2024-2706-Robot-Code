  
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;


import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.math.geometry.Pose2d;


import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.math.geometry.Pose2d;

import static frc.robot.subsystems.IntakeStatesMachine.IntakeModes.*;
import static frc.robot.subsystems.ShooterStateMachine.ShooterModes.*;

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
import frc.lib.lib2706.TunableNumber;
import frc.robot.Robot;
import frc.robot.commands.ArmFFTestCommand;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.RotateAngleToVision;
import frc.robot.commands.Shooter_Voltage;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.AutoSelector;
import frc.robot.subsystems.ArmPneumaticsSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeStatesMachine.IntakeStates.*;

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
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

  private TunableNumber shooterTargetRPM = new TunableNumber("Shooter/Target RPM", 0);
  private TunableNumber shooterDesiredVoltage = new TunableNumber("Shooter/desired Voltage", 0);
    
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

    intake.setDefaultCommand(intake.defaultIntakeCommand());
    shooter.setDefaultCommand(shooter.defaultShooterCommand(()-> intake.isNoteIn()));

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
    /* --------------- Driver Controls -------------------- */
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

    /* --------------- Operator Controls -------------------- */
    operator.y() //Turn on the shooter and get voltage from DS
      .whileTrue(Commands.runOnce(()->shooter.setMode(SHOOT_SPEAKER)))
      .whileFalse(Commands.runOnce(()->shooter.setMode(STOP_SHOOTER)));

    operator.a() //Intake the Note
      .whileTrue(Commands.runOnce(()-> intake.setMode(INTAKE)))
      .whileFalse(Commands.runOnce(()->intake.setMode(STOP_INTAKE)));    

    operator.b() //Release the Note from the back
      .whileTrue(Commands.runOnce(()-> intake.setMode(RELEASE)))
      .whileFalse(Commands.runOnce(()->intake.setMode(STOP_INTAKE)));    

    operator.x() //Drives the note into the shooter
      .whileTrue(Commands.runOnce(()-> intake.setMode(shooter.isReadyToShoot() ? SHOOT : STOP_INTAKE)))
      .whileFalse(Commands.runOnce(()->intake.setMode(STOP_INTAKE)));    

    operator.povDown() //Stops the state machine from jumping automaticly between states
      .onTrue(Commands.runOnce(()->{intake.setStateMachineOff();shooter.setStateMachineOff();}));

    operator.povUp() //Re-activates the state
      .onTrue(Commands.runOnce(()->{intake.setStateMachineOn();shooter.setStateMachineOn();}));

    operator.start() //Shoots the Note automatically 
      .onTrue(Commands.sequence(
          shooter.speedUpForSpeakerCommand(),
          intake.shootNoteCommand(),
          Commands.runOnce(()->shooter.setMode(STOP_SHOOTER))          
          ));

    /*operator.start() //Shoots the Note automatically <
      .onTrue(Commands.deadline(
        Commands.sequence(
          Commands.waitSeconds(2), 
          intake.shootNote())
          ,new Shooter_tuner(()->12)
      ));

      /**
       * operator.start().whileTrue(Commands.deadline(
      Commands.sequence(
        new IntakeControl(false), 
        new WaitCommand(0.5), 
        new IntakeControl(true).withTimeout(2)),
      new Shooter_tuner(12)
    ));
       */

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
