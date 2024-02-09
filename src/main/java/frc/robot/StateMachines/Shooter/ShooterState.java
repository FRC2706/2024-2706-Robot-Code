// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.StateMachines.Shooter;

import static frc.robot.StateMachines.Shooter.ShooterState.Modes.*;
import static frc.robot.StateMachines.Shooter.ShooterState.States.*;

import java.util.Collection;
import java.util.Collections;

import org.littletonrobotics.junction.AutoLogOutput;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.lib.lib2706.RunnableBoolean;
import frc.robot.Mechanisms.Shooter.ShooterIOValuesAutoLogged;

/** Add your docs here. */
public class ShooterState {
    private boolean isInRange = false;
    private static double distanceFromSpeaker = 0.0;

    private Modes desiredMode = IDLE;
    private States currentState = IN_IDLE;
    private static InterpolatingTreeMap<Double, Double> interpolation = new InterpolatingTreeMap<Double, Double>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());

    static {
        interpolation.put(0.0, 0.0);// Min value
        interpolation.put(0.0, 0.0);// Query
        interpolation.put(0.0, 0.0);// Max value
    }

    public static enum Modes {
        IDLE(0.0),
        PRE_HEAT(0.0),
        SHOOT_AMP(0.0),
        SHOOT_SPEAKER(interpolation.get(distanceFromSpeaker));

        double s;

        private Modes(double speeds) {
            s = speeds;
        }

        public double getDesiredSpeed() {
            return s;
        }
    }

    public static enum States {
        IN_IDLE,//-
        PRE_HEATING,//-
        PRE_HEATED,
        REACHING_SET_POINT,//-
        ON_SET_POINT,
    }

    public void setMode(Modes desiredMode) {
        this.desiredMode = desiredMode;
    }

    public double getDesiredVelocity() {
        return desiredMode.getDesiredSpeed();
    }

    public void updateDistance(double h) {
        distanceFromSpeaker = h;
    }

    public void isInRange(RunnableBoolean toRun) {
        if (toRun != null)
            isInRange = toRun.run();
        else
            isInRange = false;
    }

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
        }
        return currentState; 
    }
}
