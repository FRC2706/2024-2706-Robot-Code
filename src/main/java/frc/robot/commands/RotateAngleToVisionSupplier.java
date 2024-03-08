// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateAngleToVisionSupplier extends TeleopSwerve {
  DoubleSupplier m_supplier;
  double m_setpoint;
  
  /** Creates a new RotateAngleToVisionSupplier. */
  public RotateAngleToVisionSupplier(CommandXboxController driver, DoubleSupplier supplier) {
    super(driver);

    m_supplier = supplier;
  }

  public RotateAngleToVisionSupplier(CommandXboxController driver, String photonvisionCameraName){
    super(driver);

    DoubleSubscriber yawSub = NetworkTableInstance.getDefault()
        .getDoubleTopic(photonvisionCameraName + "/targetYaw")
        .subscribe(0, PubSubOption.periodic(0.02));

    BooleanSubscriber hasData = NetworkTableInstance.getDefault()
        .getBooleanTopic(photonvisionCameraName + "/hasTarget")
        .subscribe(false, PubSubOption.periodic(0.02));

    m_supplier = () -> {
      if (!hasData.get(false)) {
        return 0;
      }
      return Math.toRadians(-1 * yawSub.get(0));
    };
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();

    SwerveSubsystem.getInstance().resetDriveToPose();
    m_setpoint = SwerveSubsystem.getInstance().getHeading().getRadians();
  }

  @Override
  protected double calculateRotationVal() {
    m_setpoint = SwerveSubsystem.getInstance().getHeading().getRadians() + m_supplier.getAsDouble();

    return SwerveSubsystem.getInstance().calculateRotation(new Rotation2d(m_setpoint));
  }
}
