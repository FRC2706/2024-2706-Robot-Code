// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleJoystick extends Command {
  private Timer m_timer = new Timer();
  private CommandXboxController m_joystick;
  private RumbleType rumbleType;
  private double strength;
  private double duration;
  private boolean doDoubleRumble;
  private double counter;
  private double x;
  /** Creates a new RumbleJoystick. */
    /**
     * @brief Command that rumbles the joystick with two modes: Single rumble for a certain duration, DoubleRumble which splits the duration into three, rumbles on, off, on. 
     * @param joystick name of joystick to vibrate
     * @param rumbleType Type of rumble, see RumbleType definition for more details
     * @param strength Number between 0 and 1 which dictates the strength of the vibration
     * @param duration duraction vibration
     * @param doDoubleRumble true for doubleRumble, false for singleRumble
     */
  public RumbleJoystick(CommandXboxController joystick, RumbleType rumbleType, double strength, double duration, boolean doDoubleRumble) {
    counter = 0;
    x = 0;
    this.m_joystick = joystick;
    this.rumbleType = rumbleType;
    this.strength = strength;
    this.duration = duration;
    this.doDoubleRumble = doDoubleRumble;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    m_joystick.getHID().setRumble(rumbleType, strength);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (doDoubleRumble == true) {
      if (m_timer.hasElapsed(x+(duration/3))){
        x+=(duration/3);
        counter++;
      }
      if (counter ==1) {
        m_joystick.getHID().setRumble(rumbleType,0);
      }
      if (counter ==2){
        m_joystick.getHID().setRumble(rumbleType,strength);
      }
    }

  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_joystick.getHID().setRumble(rumbleType,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { //
    return m_timer.hasElapsed(duration);
  }
}
