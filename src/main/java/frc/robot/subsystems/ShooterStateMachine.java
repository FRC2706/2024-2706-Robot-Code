// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.subsystems.ShooterStateMachine.ShooterModes.*;
import static frc.robot.subsystems.ShooterStateMachine.States.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/** Add your docs here. */
public class ShooterStateMachine {
    private boolean isInRange = false;
    private static double distanceFromSpeaker = 0.0;

    private ShooterModes desiredMode = STOP_SHOOTER;
    private States currentState = IN_IDLE;

    /**
     * All possible Modes to control the shooter
    */
    public static enum ShooterModes {
        STOP_SHOOTER(0.0, 0.0),
        PRE_HEAT(4.0, 500.0),
        SHOOT_AMP(6.0, 800.0),
        SHOOT_SPEAKER(9.0, 1700.0);//MAX 2500 with 9 v

        double v, RPM;

        private ShooterModes(double voltage, double velo) {
            v = voltage;
            RPM = velo;
        }

        public double getDesiredVoltage() {
            return v;
        }

        public double getDesiredSpeedRPM() {
            return RPM;
        }
    }

    /** 
     * All possible states of the shooter
     */
    public static enum States {
        IN_IDLE,//-
        PRE_HEATING,//-
        PRE_HEATED,
        REACHING_SET_POINT,//-
        AMP_LAUNCH_READY,
        SPEAKER_LAUNCH_READY,
    }

    /**
     * Method to set the desired mode
     * @param desiredMode
     */
    public void setMode(ShooterModes desiredMode) {
        this.desiredMode = desiredMode;
    }

    /**
     * Method to get the desired voltage depending on the mode it was set
     * @return desired velocity 
     */
    public double getDesiredVoltage() {
        return desiredMode.getDesiredVoltage();
    }
    /**
     * Method to get the desired velocity depending on the mode it was set
     * @return desired velocity 
     */
    public double getDesiredVelocityRPM() {
        return desiredMode.getDesiredSpeedRPM();
    }

    /**
     * Method for logging values
     */
    public ShooterModes getDesiredMode(){
        return desiredMode;
    }

    /**
     * Method for getting current state
     */
    public States getCurrentState(){
        return currentState;
    }

    /**
     * Method used to get the best speed depending on the distance of the robot from aprilTag
     * @param h
     */
    public void updateDistance(double h) {
        distanceFromSpeaker = h;
    }

    /**
     * Method used by the state machine to switch from states
     * @param toRun
     */
    public void isInRange(BooleanSupplier toRun) {
        if (toRun != null)
            isInRange = toRun.getAsBoolean();
        else
            isInRange = false;
    }

    /**
     * Method that will automatically change the states of the machine
     * This should be called every loop cycle
     */
    public void updateState() {
        switch (desiredMode) {
            case STOP_SHOOTER:
                currentState = IN_IDLE;
                break;
            case PRE_HEAT:
                currentState = isInRange ? PRE_HEATED : PRE_HEATING;
                break;
            case SHOOT_AMP:
                currentState = isInRange ? AMP_LAUNCH_READY: REACHING_SET_POINT;
                break;
            case SHOOT_SPEAKER:
                currentState = isInRange ? SPEAKER_LAUNCH_READY: REACHING_SET_POINT;
                break;
            default:
                break;
        }
    }
}