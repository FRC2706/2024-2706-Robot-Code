// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateAngleToVisionSupplier extends TeleopSwerve {
  ProfiledPIDController pid = new ProfiledPIDController(2.3, 0, 0.8, 
                                        new TrapezoidProfile.Constraints(4 * Math.PI, 8 * Math.PI)); //pid to be tested
  DoubleSupplier m_supplier;
  double setpoint;
  /** Creates a new RotateAngleToVisionSupplier. */
  public RotateAngleToVisionSupplier(SwerveSubsystem s_Swerve, CommandXboxController driver, DoubleSupplier supplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(s_Swerve, driver);

    this.m_supplier = supplier;

    pid.setTolerance(0.1);
    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  public RotateAngleToVisionSupplier(SwerveSubsystem s_Swerve, CommandXboxController driver, DoubleSubscriber subscriber){
    this(s_Swerve, driver, ()-> subscriber.getAsDouble()); 
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();

    setpoint = SwerveSubsystem.getInstance().getHeading().getRadians();
  }

  @Override
  public double calculateRotationVal() {
    double value = m_supplier.getAsDouble();
    if (value != -99 && Math.abs(value) < 30) {
      setpoint = SwerveSubsystem.getInstance().getHeading().getRadians() + Math.toRadians(value*-1);
    }
    return(pid.calculate(SwerveSubsystem.getInstance().getHeading().getRadians(), setpoint));
  }
}
