// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.DriveMechanisms;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/** Add your docs here. */
public class ModulesIOSparkMax implements ModulesIO{
// Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogInput turnAbsoluteEncoder;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModulesIOSparkMax(int index) {
    switch (index) {
      case 0:
        driveSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(0);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 1:
        driveSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(1);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 2:
        driveSparkMax = new CANSparkMax(5, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(6, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(2);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 3:
        driveSparkMax = new CANSparkMax(7, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(8, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(3);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkMax.setSmartCurrentLimit(40);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateValues(ModulesIOValues values) {
    values.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    values.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    values.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    values.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    values.turnAbsolutePosition =
        new Rotation2d(
                turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset);
    values.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    values.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    values.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    values.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}