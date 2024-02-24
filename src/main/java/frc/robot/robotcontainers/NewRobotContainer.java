  
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;


import static frc.robot.subsystems.IntakeStates.Modes.RELEASE;
import static frc.robot.subsystems.IntakeStates.Modes.SHOOT;
import static frc.robot.subsystems.IntakeStates.Modes.STOP;


import edu.wpi.first.math.geometry.Pose2d;


import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.math.geometry.Pose2d;

import static frc.robot.subsystems.IntakeStatesVoltage.Modes.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.lib2706.TunableNumber;
import frc.robot.Config.Swerve.TeleopSpeeds;
import frc.robot.Robot;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.RotateAngleToVision;
import frc.robot.commands.Shooter_tuner;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.subsystems.ArmPneumaticsSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

  private TunableNumber shooterTargetRPM = new TunableNumber("Shooter/Target RPM", 0);
  private TunableNumber shooterDesiredVoltage = new TunableNumber("Shooter/desired Voltage", 0);
    
  String tableName = "SwerveChassis";
  //private NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable(tableName);
  //private IntegerEntry entryAutoRoutine;

  // AutoSelector m_autoSelector;

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

    intake.setDefaultCommand(intake.autoIntake());

    //entryAutoRoutine = swerveTable.getIntegerTopic("Auto Selector ID").getEntry(0);
    //entryAutoRoutine.setDefault(0);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() { 
    
    
    /* Driver Controls */
    
    driver.back().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d(0)));
    /* --------------- Driver Controls -------------------- */
  
    //driver.back().whileTrue(SwerveSubsystem.getInstance().setLockWheelsInXCommand());
    driver.start().whileTrue(new RotateAngleToVision(s_Swerve, driver, 0));
    driver.leftBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW))).onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));
    // driver.

    /* --------------- Operator Controls -------------------- */
    operator.y() //Manually turn on the shooter and get voltage from DS
      .whileTrue(new Shooter_tuner(()->shooterDesiredVoltage.get()));

   // operator.y().whileTrue (new ArmFFTestCommand(operator, 3, true) );

    //operator.a() //Intake the Note
      //.whileTrue(Commands.runOnce(()-> intake.setMode(INTAKE)))
      //.whileFalse(Commands.runOnce(()->intake.setMode(STOP)));    
      
    operator.b() //Release the Note from the back
      .whileTrue(Commands.runOnce(()-> intake.setMode(RELEASE)))
      .whileFalse(Commands.runOnce(()->intake.setMode(STOP)));    

    operator.x() //Drives the note into the shooter
      .whileTrue(Commands.runOnce(()-> intake.setMode(SHOOT)))
      .whileFalse(Commands.runOnce(()->intake.setMode(STOP)));    

    //operator.start() //Shoots the Note automatically 
      //.onTrue(Commands.deadline(
        //Commands.sequence(
          //Commands.waitSeconds(2), 
          //intake.shootNote())
          //,new Shooter_tuner(12)
      //));
    operator.start() //Shoots the Note automatically 
      .onTrue(Commands.deadline(
        Commands.sequence(
          Commands.waitSeconds(2), 
          intake.shootNote())
          ,new Shooter_tuner(()->12)
      ));

      //    operator.a() 
      // .whileTrue(new MakeIntakeMotorSpin(9.0, 0));

    //     operator.start().whileTrue(Commands.deadline(
    //   Commands.sequence(
    //     new IntakeControl(false), 
    //     new WaitCommand(0.5), 
    //     new IntakeControl(true).withTimeout(2)),
    //   new Shooter_tuner(5)
    // ));

    //   operator.start().whileTrue(Commands.parallel(
    //   Commands.sequence(
    //     new IntakeControl(false), 
    //     new IntakeControl(true).withTimeout(2)),
    //   new Shooter_tuner(5)
    // ));
       

    //turns brakes off
    operator.rightBumper().onTrue(Commands.runOnce(() -> ArmPneumaticsSubsystem.getInstance().controlBrake(false, true)));

    //turns brakes on
    operator.rightTrigger().onTrue(Commands.runOnce(() -> ArmPneumaticsSubsystem.getInstance().controlBrake(true, true)));
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *leop
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutoRoutines().getAutonomousCommand(6);
  }
}
