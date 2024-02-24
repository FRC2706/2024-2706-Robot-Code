// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.subsystems.IntakeStatesVoltage.Modes.*;
import static frc.robot.subsystems.IntakeStatesVoltage.States.*;

import frc.lib.lib2706.RunnableBoolean;

/** Add your docs here. */
public class IntakeStatesVoltage {
    private Boolean isForntActive = false;
    private Boolean isBackActive = false;
    private Boolean isCenterActive = null;

    private Modes desiredMode = STOP;
    private States currentState = EMPTY_IDLE;

    /**
     * All possible Modes to control the shooter
    */
    public static enum Modes {
        STOP(0.0),
        INTAKE(9.0),
        POSITION_NOTE(0.7),
        BACK_NOTE(-0.7),
        RELEASE(-9.0),
        SHOOT(9.0);

        double v;

        private Modes(double voltage) {
            v = voltage;
        }

        public double getDesiredVoltage() {
            return v;
        }
    }

    /** 
     * All possible states of the shooter
     */
    public static enum States {
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
    public void setMode(Modes desiredMode) {
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
    public Modes getDesiredMode(){
        return desiredMode;
    }

    /**
     * Method for checking current state of intake
     */
    public States getCurrentState(){
        return currentState;
    }

    /**
     * Method used by the state machine to switch from states
     * This should be called every loop cycle
     * @param toRun First Argument must be the Back Sensor, Second Front, Third the center
     */
    public void updateSensors(RunnableBoolean... toRun) {
        if(toRun.length > 3 || toRun == null){
            throw new IllegalArgumentException("No sensors found, or more than 3");
        }
        
        if(toRun.length >= 2){
            isForntActive = toRun[1].run();
        }

        isBackActive = toRun[0].run();

        if(toRun.length == 3){
            isCenterActive = toRun[2].run();
        }
        
    }

    /**
     * Method that will automatically change the states of the machine
     */
    public void updateStates() {
        switch (desiredMode) {
            //Stop from moving
            case STOP: 
                if(isBackActive){
                    setMode(BACK_NOTE); 
                    break;
                }

                if(isCenterActive == null){
                    if(isForntActive)currentState = NOTE_ENTERING_IDLE;//This state would be affected if suddenly stopped
                }else if(isCenterActive){
                    currentState = NOTE_IN_POS_IDLE;
                }else if(isForntActive){
                    currentState =  NOTE_ENTERING_IDLE;
                }else currentState = EMPTY_IDLE; 
            break;
             
            //Intake the Note
            case INTAKE:
                if(isBackActive){
                    setMode(BACK_NOTE); 
                    break;
                }

                if(isCenterActive){//Check if this works now
                    setMode(STOP);
                }

                if(isForntActive && !currentState.equals(NOTE_ENTERING_IDLE)){ //compare with NOTE_ENTERING_IDLE;
                    currentState = NOTE_ENTERING;
                    setMode(POSITION_NOTE);
                } else currentState = INTAKING;
            break;

            //Position the note once its being intaked
            case POSITION_NOTE:
            currentState = POSITIONING_NOTE;
                if(isCenterActive != null){
                    if(isCenterActive)setMode(STOP);
                }else if(isBackActive){
                    setMode(BACK_NOTE); 
                }
            break;

            //Release the note
            case RELEASE:
                currentState = RELEASING;
            break;

            //Back the note if overshooted
            case BACK_NOTE:
                //If we dont have a front intake, this will not work
                if(!isBackActive && isForntActive)setMode(STOP);
                else currentState = BACKING_NOTE;
            break;

            //Shooting the Note
            case SHOOT:
            if(isCenterActive == null){
                if(!isForntActive && !isBackActive)
                currentState = SHOOTED;
                setMode(STOP);
                break;
            }else if(!isBackActive && !isCenterActive && !isForntActive){
                currentState = SHOOTED;
                setMode(STOP);
                break;
            }else currentState = SHOOTING;
            break;

            //Default
            default:
            setMode(STOP);
                break;
        }
    }
}