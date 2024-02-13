// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class ArmFFTestCommand extends CommandBase {
  private CommandXboxController m_joystick;
  private double m_maxExtraVolts;
  private boolean m_enableBot;

  /** Creates a new ArmFFTestCommand. */
  public ArmFFTestCommand(CommandXboxController joystick, double maxExtraVolts, boolean enableBottom) {
    this.m_joystick = joystick;
    this.m_maxExtraVolts = maxExtraVolts;
    m_enableBot = enableBottom;
    addRequirements(ArmSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_enableBot) {
      ArmSubsystem.getInstance().setArmIdleMode(IdleMode.kCoast);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickValue = m_joystick.getRawAxis(XboxController.Axis.kLeftX.value);
    joystickValue = MathUtil.applyDeadband(joystickValue, 0.15);

    if (m_enableBot) {
      ArmSubsystem.getInstance().testFeedForward(joystickValue * m_maxExtraVolts);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.getInstance().stopMotors();
    ArmSubsystem.getInstance().setArmIdleMode(IdleMode.kBrake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}