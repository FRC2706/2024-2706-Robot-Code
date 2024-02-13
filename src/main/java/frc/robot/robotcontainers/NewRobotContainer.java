  
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config.Swerve.TeleopSpeeds;
import frc.robot.Robot;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.TeleopSwerve;
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

  /* Create Subsystems in a specific order */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public NewRobotContainer() {
    // Setup default commands
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            driver,
            TeleopSpeeds.MAX
        )
    );
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() {
    
    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController operator = new CommandXboxController(1);
    /* Driver Controls */
    //driver.a().whileTrue(new MakeIntakeMotorSpin(1.0,300));
    driver.a()
    .whileTrue(Commands.run(()->{intake.setVoltage(10);}, intake))
    .whileFalse(Commands.run(()->{intake.setVoltage(0);}, intake));

    
    driver.start().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d(0)));
    driver.back().whileTrue(SwerveSubsystem.getInstance().setLockWheelsInXCommand());
    driver.leftBumper().whileTrue(new TeleopSwerve(
        s_Swerve,
        driver,
        TeleopSpeeds.SLOW
    ));
    /* Operator Controls */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
