// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {
    @AutoLog
    class ArmIOValues {
        public double armPositionRads = 0.0;
        public double armAbsoluteEncoderPosRad = 0.0;
        public double armVelocityRadsPerSec = 0.0;

        //In case we have two motors on the gearbox
        public double[] armAppliedVolts = new double[] {};
        public double[] armCurrentAmps = new double[] {};
        public double[] armTempCelcius = new double[] {};
    }

    default void updateValues(ArmIOValues inputs) {
    }

    /** Run to setpoint angle in radians */
    default void setSetpoint(double setpointRads) {
    }

    /** Run motors at volts */
    default void setVolts(double volts) {
    }

    /** Set brake mode enabled */
    default void setBrakeMode(boolean enabled) {
    }

    /** Set FF values */
    default void setFF(double ff) {
    }

    /** Set PID values */
    default void setPID(double p, double i, double d) {
    }
    
    /** Stops motors */
    default void stop() {
    }
}