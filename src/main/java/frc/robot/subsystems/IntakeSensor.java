// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSensor extends SubsystemBase {
  private final DigitalInput limitSwitch; // DigitalInput instance

  public IntakeSensor() {
    // Initialize DigitalInput on channel 5
    limitSwitch = new DigitalInput(7);
  }

  @Override
  public void periodic() {
    boolean isSwitchPressed = limitSwitch.get(); // You can use the limitSwitch in your periodic code
    System.out.println("Limit Switch Status: " + isSwitchPressed);
  }
}
