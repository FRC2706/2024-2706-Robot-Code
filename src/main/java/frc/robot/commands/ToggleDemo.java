// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BlingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleDemo extends InstantCommand {
  private static boolean toggleDemo = true;
  public BlingSubsystem bling = BlingSubsystem.getINSTANCE()
  public ToggleDemo() {
    toggleDemo =! toggleDemo;
    if (bling != null)
      addRequirements(bling);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (toggleDemo == true) {
      bling.setYellow();
    }
    else { 
      bling.setHoneydew();
    }
    
  }
}
