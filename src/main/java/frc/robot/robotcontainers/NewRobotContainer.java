
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config;
import frc.robot.Config.Swerve.TeleopSpeeds;
import frc.robot.Mechanisms.DriveMechanisms.GyroIO;
import frc.robot.Mechanisms.DriveMechanisms.GyroIOPigeon;
import frc.robot.Mechanisms.DriveMechanisms.ModulesIO;
import frc.robot.Mechanisms.DriveMechanisms.ModulesIOSim;
import frc.robot.Mechanisms.DriveMechanisms.ModulesIOSparkMax;
import frc.robot.Mechanisms.Intake.IntakeIO;
import frc.robot.Mechanisms.Intake.IntakeIOSparkMax;
import frc.robot.Mechanisms.Shooter.ShooterIO;
import frc.robot.Mechanisms.Shooter.ShooterIOSparkMax;
import frc.robot.StateMachines.Drive.Drive;
import frc.robot.StateMachines.Intake.Intake;
import frc.robot.StateMachines.Shooter.Shooter;
//import frc.robot.Mechanisms.SwerveSubsystem;
import frc.robot.Robot;
import frc.robot.commands.Shooter_tuner;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.auto.AutoRoutines;

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
  //private final CommandJoystick operator = new CommandJoystick(1);
  private final Shooter shooter;
  private final Intake intake;

  // private final SwerveSubsystem s_Swerve = SwerveSubsystem.getInstance();
  /* Create Subsystems in a specific order */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public NewRobotContainer() {
    switch (Config.LoggingConstants.currentMode) {
      case REAL:
      case REPLAY:
        shooter = new Shooter(new ShooterIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        break;
    case SIM:
        //System.out.println("[Init] Simulation Created");
        shooter = new Shooter(new ShooterIO(){});
        intake = new Intake(new IntakeIO(){});

        break;
      default:
        shooter = new Shooter(new ShooterIO() {});
        intake = new Intake(new IntakeIO(){});

        break;
    }

    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() {
    driver.a().whileTrue(new Shooter_tuner(shooter));
    driver.b()
    .whileTrue(Commands.run(()->{shooter.setSetV(Shooter.TUNING_MODE); System.out.println("Shooting");}, shooter))
    .whileFalse(Commands.run(()->{shooter.setSetV(0);}, shooter)); 
    //operator.button(1).whileTrue(new Shooter_tuner(shooter));
    driver.x()
    .whileTrue(Commands.run(()->{intake.setSetV(Intake.TUNING_MODE); System.out.println("intaking");}, intake))
    .whileFalse(Commands.run(()->{intake.setSetV(0);}, intake)); 
    driver.y()
    .whileTrue(Commands.run(()->{intake.setSetV(-10); System.out.println("intaking");}, intake))
    .whileFalse(Commands.run(()->{intake.setSetV(0);}, intake)); 
    //operator.button(1).whileTrue(new Shooter_tuner(shooter));

    /*
     * Driver Controls
     * driver.start().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new
     * Rotation2d(0)));  
     * driver.back().whileTrue(SwerveSubsystem.getInstance().setLockWheelsInXCommand
     * ());
     * driver.leftBumper().whileTrue(new TeleopSwerve(
     * s_Swerve,
     * driver,
     * TeleopSpeeds.SLOW
     * ));
     * /* Operator Controls
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
