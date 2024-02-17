// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO{
    private final FlywheelSim m_motor = new FlywheelSim(DCMotor.getNEO(1), 1, 40);
    private final PIDController m_controller = new PIDController(0, 0, 0);

    private double feedForward = 0.0;
    private double appliedVolts = 0.0;
    private Double setPoint = null;

    @Override
    public void updateValues(ShooterIOValues values) {
        if(setPoint != null){
            appliedVolts = m_controller.calculate(m_motor.getAngularVelocityRPM(), setPoint) + feedForward;
            m_motor.setInputVoltage(MathUtil.clamp(appliedVolts, -12, 12));
        }

        values.flywheelAppliedVolts = appliedVolts;
        values.flywheelOutputCurrent = m_motor.getCurrentDrawAmps();
        values.flywheelPositionRotations = 0.0;
        values.flywheelVelocityRPM = m_motor.getAngularVelocityRPM();
    }

    @Override
    public void setCharacterizationVoltage(double volts) {
        setPoint = null;
        appliedVolts = MathUtil.clamp(volts, -12, 12);
        m_motor.setInputVoltage(appliedVolts);
    }

    @Override
    public void setFF(double volts) {
        feedForward = volts;
    }

    @Override
    public void setPID(double p, double i, double d) {
        m_controller.setPID(p, i, d);
    }

    @Override
    public void setRPM(double rpm) {
        setPoint = rpm;
    }

    @Override
    public void setVoltage(double volts) {
        setPoint = null;
        appliedVolts = MathUtil.clamp(volts, -12, 12);
        m_motor.setInputVoltage(appliedVolts);
    }

    @Override
    public void stop() {
        setRPM(0);
    }
}