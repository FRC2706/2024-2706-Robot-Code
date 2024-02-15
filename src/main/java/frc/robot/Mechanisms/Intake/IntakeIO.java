// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.Intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
  @AutoLog
  class IntakeIOValues {
    public double flywheelPositionRotations = 0.0;
    public double flywheelVelocityRPM = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double flywheelOutputCurrent = 0.0;
  }

  /** Update inputs */
  default void updateValues(IntakeIOValues values) {}

  default void setRPM(double rpm) {}

  default void setShooterBrakeMode(boolean enabled) {}

  default void setCharacterizationVoltage(double volts) {}
  
  default void setVoltage(double volts) {}

  default void setPID(double p, double i, double d) {}

  default void setFF(double volts) {}

  default void stop() {}
}