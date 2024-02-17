// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static frc.robot.subsystems.Shooter.ShooterState.Modes.*;
import static frc.robot.subsystems.Shooter.ShooterState.States.*;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.lib.lib2706.RunnableBoolean;

/** Add your docs here. */
public class ShooterState {
    private boolean isInRange = false;
    private static double distanceFromSpeaker = 0.0;

    private Modes desiredMode = IDLE;
    private States currentState = IN_IDLE;
    private static InterpolatingTreeMap<Double, Double> interpolation = new InterpolatingTreeMap<Double, Double>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());

    //Initialize the values of the interpolation as soon as the code is launched
    static {
        interpolation.put(0.0, 0.0);// Min value
        interpolation.put(0.0, 0.0);// Query
        interpolation.put(0.0, 0.0);// Max value
    }

    /**
     * All possible Modes to control the shooter
    */
    public static enum Modes {
        IDLE(0.0),
        PRE_HEAT(0.0),
        SHOOT_AMP(0.0),
        SHOOT_SPEAKER(interpolation.get(distanceFromSpeaker)),
        RUN_BY_VOLTAGE(0.0);

        double s;

        private Modes(double speeds) {
            s = speeds;
        }

        public double getDesiredSpeed() {
            return s;
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
        ON_SET_POINT,
        IS_VOLTAGE_RUN
    }

    /**
     * Method to set the desired mode
     * @param desiredMode
     */
    public void setMode(Modes desiredMode) {
        this.desiredMode = desiredMode;
    }

    /**
     * Method to get the velocity depending on the mode it was set
     * @return desired velocity 
     */
    public double getDesiredVelocity() {
        return desiredMode.getDesiredSpeed();
    }

    /**
     * Method for logging values
     */
    public Modes getDesiredMode(){
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
    public void isInRange(RunnableBoolean toRun) {
        if (toRun != null)
            isInRange = toRun.run();
        else
            isInRange = false;
    }

    /**
     * Method that will automatically change the states of the machine
     * This should be called every loop cycle
     */
    public States updateState() {
        switch (desiredMode) {
            case IDLE:
                currentState = IN_IDLE;
                break;
            case PRE_HEAT:
                currentState = isInRange ? PRE_HEATED : PRE_HEATING;
                break;
            case SHOOT_AMP:
                currentState = isInRange ? ON_SET_POINT: REACHING_SET_POINT;
                break;
            case SHOOT_SPEAKER:
                currentState = isInRange ? ON_SET_POINT: REACHING_SET_POINT;
                break;
            case RUN_BY_VOLTAGE:
                currentState = IS_VOLTAGE_RUN;
                break;
            default:
                break;
        }
        return currentState; 
    }
}
