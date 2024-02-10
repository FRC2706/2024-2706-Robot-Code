// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.StateMachines.Arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib6328.LoggedTunableNumber;
import frc.robot.Config;
import frc.robot.Mechanisms.Arm.ArmIO;
import frc.robot.Mechanisms.Arm.ArmIOValuesAutoLogged;

public class Arm extends SubsystemBase {
  private ArmIO armIO;
  private ArmIOValuesAutoLogged armValues;
  // private ExponentialProfile s;//give it a check
  private double lastVel = 0;

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(Config.ArmConfig.MAX_VEL,
      Config.ArmConfig.MAX_ACCEL);
  private final ProfiledPIDController m_ProfiledPIDController = new ProfiledPIDController(1.6, 0.002, 40, m_constraints,
      0.02);

  private LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", Config.ShooterConstants.kP);
  private LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", Config.ShooterConstants.kI);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", Config.ShooterConstants.kD);
  private LoggedTunableNumber kFF = new LoggedTunableNumber("Arm/kFF", Config.ShooterConstants.kFF);
  private LoggedTunableNumber setPointRPM = new LoggedTunableNumber("Arm/setPointAngle", 0);

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    System.out.println("[Init]Creating Arm");
    armIO = io;
    armIO.setPID(kP.get(), kI.get(), kD.get());
    armIO.setFF(kFF.get());
  }

  public void setSetPoint(double angle) {
    angle = MathUtil.clamp(angle, Math.toRadians(Config.ArmConfig.MIN_ARM_ANGLE_DEG),
        Math.toRadians(Config.ArmConfig.MAX_ARM_ANGLE_DEG));

    double targetPos = m_ProfiledPIDController.calculate(Units.radiansToDegrees(armValues.armAbsoluteEncoderPosRad), angle);
    armIO.setSetpoint(targetPos);
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(hashCode(),
        () -> armIO.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(hashCode(),
        () -> armIO.setFF(kFF.get()), kFF);

    armIO.updateValues(armValues);
    Logger.processInputs("Shooter", armValues);

    if (DriverStation.isDisabled()) {
      armIO.stop();
    }

  }
}
