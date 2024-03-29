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
    private static InterpolatingTreeMap<Double, Double> interpolation = new InterpolatingTreeMap<Double, Double>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());

    private ShooterModes desiredMode = STOP_SHOOTER;
    private States currentState = IN_IDLE;

    //Initialize the values of the interpolation as soon as the code is launched
    public ShooterStateMachine() {
        interpolation.put(0.0, 0.0);// Min value
        interpolation.put(0.0, 0.0);// Query
        interpolation.put(0.0, 0.0);// Max value
    }

    /**
     * All possible Modes to control the shooter
    */
    public static enum ShooterModes {
        STOP_SHOOTER(0.0, 0.0),
        PRE_HEAT(4.0, 500.0),
        INTERPOLATED_SHOOT(0.0, interpolation.get(distanceFromSpeaker)),
        SHOOT_AMP(6.0, 800.0),
        CLOSE_SHOOT_SPEAKER(9.0, 3200.0);

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
        INTERPOLATED_LAUNCH_READY
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
     * Method that updates the distance from the speaker to the 
     * @param d
     */
    public void updateDistance(double d) {
        distanceFromSpeaker = d;
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
            
            case CLOSE_SHOOT_SPEAKER:
                currentState = isInRange ? SPEAKER_LAUNCH_READY: REACHING_SET_POINT;
            break;
            
            case INTERPOLATED_SHOOT:
                currentState = isInRange ? INTERPOLATED_LAUNCH_READY: REACHING_SET_POINT;
            break;
            
            default:
                setMode(STOP_SHOOTER);
                break;
        }
    }
}