// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.lib2706;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SelectByAllianceCommand extends Command {
  private Command blueCommand, redCommand;
  private Command commandToRun;

  /**
   * Run either a blue command or a red command based on which
   * alliance is reported from the driver station.
   * Note: Combines both requirements into this one command.
   * 
   * @param blue The command to run on the blue alliance.
   * @param red The command to run on the red alliance.
   */
  public SelectByAllianceCommand(Command blue, Command red) {
    blueCommand = blue;
    redCommand = red;

    for (Subsystem s: blue.getRequirements()) {
      addRequirements(s);
    }

    for (Subsystem s: red.getRequirements()) {
      addRequirements(s);
    }
  }

  /**
   * Get the alliance colour from the DriverStation.
   * 
   * @return True for blue, false for red.
   */
  private boolean isBlueAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return true;
    }
    return alliance.get() == DriverStation.Alliance.Blue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (isBlueAlliance()) {
      commandToRun = blueCommand;
    } else {
      commandToRun = redCommand;
    }
    commandToRun.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    commandToRun.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    commandToRun.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandToRun.isFinished();
  }
}
