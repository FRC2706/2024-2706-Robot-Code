// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.subsystems.IntakeStateMachine.IntakeModes.*;
import static frc.robot.subsystems.IntakeStateMachine.IntakeStates.*;

import java.nio.file.attribute.PosixFilePermission;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class IntakeStateMachine {
    private Boolean isForntActive = false;
    private Boolean isBackActive = false;
    private Boolean isCenterActive = null;

    private IntakeModes desiredMode = STOP_INTAKE;
    private IntakeStates currentState = EMPTY_IDLE;

    /**
     * All possible Modes to control the shooter
    */
    public static enum IntakeModes {
        STOP_INTAKE(0.0),
        INTAKE(9.0),
        POSITION_NOTE(2.0),
        BACK_NOTE(-2.0),
        RELEASE(-9.0),
        SHOOT(9.0);

        double v;

        private IntakeModes(double voltage) {
            v = voltage;
        }

        public double getDesiredVoltage() {
            return v;
        }
    }

    /** 
     * All possible states of the shooter
     */
    public static enum IntakeStates {
        EMPTY_IDLE,
        INTAKING,
        RELEASING,
        BACKING_NOTE,
        POSITIONING_NOTE,
        NOTE_ENTERING,//Front active [Intake, MOVING]
        NOTE_ENTERING_IDLE,//Front active [Intake, IDLE]
        NOTE_IN_POS_IDLE, //Front (Center) active, back inactive [Intake, IDLE]
        NOTE_OVERINTAKED,//Front (Center) and back active [Intake, MOVING]
        SHOOTING,
        SHOOTED,
    }

    /**
     * Method to set the desired mode
     * @param desiredMode
     */
    public void setMode(IntakeModes desiredMode) {
        this.desiredMode = desiredMode;
    }

    /**
     * Method to get the velocity depending on the mode it was set
     * @return desired velocity 
     */
    public double getDesiredVoltage() {
        return desiredMode.getDesiredVoltage();
    }

    /**
     * Method for logging values
     */
    public IntakeModes getDesiredMode(){
        return desiredMode;
    }

    /**
     * Method for checking current state of intake
     */
    public IntakeStates getCurrentState(){
        return currentState;
    }

    /**
     * Method used by the state machine to switch from states
     * This should be called every loop cycle
     * @param toRun First Argument must be the Back Sensor, Second Center, Third the Front
     */
    public void updateSensors(BooleanSupplier... toRun) {
        if(toRun.length > 3 || toRun == null){
            throw new IllegalArgumentException("No sensors found, or more than 3");
        }
        
        if(toRun.length >= 2){
            isCenterActive = toRun[1].getAsBoolean();
        }

        isBackActive = toRun[0].getAsBoolean();

        if(toRun.length == 3){
            isForntActive = toRun[2].getAsBoolean();
        }
        
    }

    /**
     * Method that will automatically change the states of the machine
     */
    public void updateStates() {
        switch (desiredMode) {
            //Stop from moving
            case STOP_INTAKE: 
                if(isBackActive && isCenterActive){
                    setMode(BACK_NOTE); 
                    break;
                }

                if(isCenterActive && !isBackActive){
                    currentState = NOTE_IN_POS_IDLE;
                }else currentState = EMPTY_IDLE; 
            break;
             
            //Intake the Note
            case INTAKE:
                if(isCenterActive){//Check if this works now
                    setMode(STOP_INTAKE);
                } else currentState = INTAKING;
            break;

            //Release the note
            case RELEASE:
                currentState = RELEASING;
            break;

            //Back the note if overshooted
            case BACK_NOTE:
                //If we dont have a front intake, this will not work
                if(!isBackActive && isCenterActive)setMode(STOP_INTAKE);
                else if(!isBackActive && !isCenterActive)setMode(POSITION_NOTE);
                else currentState = BACKING_NOTE;
            break;

            //Position the note once its being intaked
            case POSITION_NOTE:
                if(isCenterActive){
                    setMode(STOP_INTAKE); 
                }else currentState = POSITIONING_NOTE;

            break;

            //Shooting the Note
            case SHOOT:
                if(!isBackActive && !isCenterActive){
                    currentState = SHOOTED;
                    setMode(STOP_INTAKE);
                    break;
                }else currentState = SHOOTING;
            break;

            //Default
            default:
                setMode(STOP_INTAKE);
            break;
        }
    }
} 