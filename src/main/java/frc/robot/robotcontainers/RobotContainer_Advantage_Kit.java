// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.Mechanisms.DriveMechanisms.GyroIO;
import frc.robot.Mechanisms.DriveMechanisms.GyroIOPigeon;
import frc.robot.Mechanisms.DriveMechanisms.ModulesIO;
import frc.robot.Mechanisms.DriveMechanisms.ModulesIOSim;
import frc.robot.Mechanisms.DriveMechanisms.ModulesIOSparkMax;
import frc.robot.Mechanisms.Shooter.ShooterIO;
import frc.robot.Mechanisms.Shooter.ShooterIOSim;
import frc.robot.Mechanisms.Shooter.ShooterIOSparkMax;
import frc.robot.StateMachines.Drive.Drive;
import frc.robot.StateMachines.Shooter.Shooter;

/** Add your docs here. */
public class RobotContainer_Advantage_Kit extends RobotContainer {
  private CommandXboxController driver_controller = new CommandXboxController(0);

  private Drive drive;
  private Shooter shooter;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer_Advantage_Kit() {
    switch (Config.LoggingConstants.currentMode) {
      case REAL:
      case REPLAY:
        drive = new Drive(
            new GyroIOPigeon(),
            new ModulesIOSparkMax(0),
            new ModulesIOSparkMax(1),
            new ModulesIOSparkMax(2),
            new ModulesIOSparkMax(3));

        shooter = new Shooter(new ShooterIOSparkMax());
        break;
      case SIM:
        drive = new Drive(
            new GyroIOPigeon(),
            new ModulesIOSim(),
            new ModulesIOSim(),
            new ModulesIOSim(),
            new ModulesIOSim());

        shooter = new Shooter(new ShooterIOSim());
        break;

      default:
        drive = new Drive(
            new GyroIO() {},
            new ModulesIO() {},
            new ModulesIO() {},
            new ModulesIO() {},
            new ModulesIO() {});

        shooter = new Shooter(new ShooterIO(){});
        break;
    }
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
