// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import frc.lib.lib2706.ProfiledPIDFFController;

import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class ArmIOSparkMax {
    private CANSparkMax m_motor; // bottom SparkMax motor controller
    private SparkAbsoluteEncoder m_absoluteEncoder;
    private SparkPIDController m_pidController;

      ProfiledPIDFFController m_profiledFFController = new ProfiledPIDFFController();

}
