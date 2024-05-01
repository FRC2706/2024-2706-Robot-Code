// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.subsystems.IntakeStateMachine.IntakeModes.*;
import static frc.robot.subsystems.IntakeStateMachine.IntakeStates.*;
import static frc.robot.subsystems.ShooterStateMachine.States.SPEAKER_LAUNCH_READY;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.subsystems.IntakeStateMachine.IntakeModes;
import frc.robot.subsystems.IntakeStateMachine.IntakeStates;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase{
    private CANSparkMax m_intake;
    private boolean stateFulControl = true;
    private IntakeStateMachine intakeStates = new IntakeStateMachine();

    private DigitalInput frontSensor;//  -> 0 
    private DigitalInput centerSensor;// -> 1
    private DigitalInput backSensor;//   -> 2

    private Debouncer frontSensorDebouncer;
    private Debouncer centerSensorDebouncer;
    private Debouncer backSensorDebouncer;
    private Debouncer backSensorLongDebouncer;

    private BooleanPublisher frontSensorPub;
    private BooleanPublisher centerSensorPub;
    private BooleanPublisher backSensorPub;
    private BooleanPublisher backSensorLongPub;
    private StringPublisher statesPub;

    private boolean frontSensorResult;
    private boolean centerSensorResult;
    private boolean backSensorResult;
    private boolean backSensorLongResult;

    private static IntakeSubsystem instance;
    public static IntakeSubsystem getInstance() {
        if (instance == null)
            instance = new IntakeSubsystem();
        return instance;
    }

    private IntakeSubsystem() {
        System.out.println("[Init]Creating Intake");
        m_intake = new CANSparkMax(Config.Intake.INTAKE, MotorType.kBrushless);
        m_intake.restoreFactoryDefaults();
        m_intake.setInverted(true);
        m_intake.setSmartCurrentLimit(70);
        m_intake.setIdleMode(IdleMode.kBrake);
        m_intake.enableVoltageCompensation(10);

        frontSensor = new DigitalInput(Config.Intake.frontSensor);
        centerSensor = new DigitalInput(Config.Intake.centerSensor);
        backSensor = new DigitalInput(Config.Intake.backSensor);

        frontSensorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        centerSensorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        backSensorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        backSensorLongDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kBoth);

        NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Intake");
        statesPub = intakeTable.getStringTopic("Intake's Current State").publish(PubSubOption.periodic(0.02));
        frontSensorPub = intakeTable.getBooleanTopic("front sensor result").publish(PubSubOption.periodic(0.02));
        centerSensorPub = intakeTable.getBooleanTopic("center sensor result").publish(PubSubOption.periodic(0.02));
        backSensorPub = intakeTable.getBooleanTopic("back sensor result").publish(PubSubOption.periodic(0.02));
        backSensorLongPub = intakeTable.getBooleanTopic("back sensor result").publish(PubSubOption.periodic(0.02));

        ErrorTrackingSubsystem.getInstance().register(m_intake);

        // Must be the last thing in the constructor
        burnFlash();
    }

    /**
     * Save the configurations from flash to EEPROM.
     */
    private void burnFlash() {
        try {
        Thread.sleep(200);
        } 
        catch (Exception e) {}

        m_intake.burnFlash();
    }

    public boolean isFrontSensorActive(){
        return frontSensorResult;
    }

    public boolean isCenterSensorActive(){
        return centerSensorResult;
    }
    
    public boolean isBackSensorActive(){
        return backSensorResult;
    }

    public boolean isBackSensorLongActive(){
        return backSensorLongResult;
    }

    public void setVoltage(double voltage){
        m_intake.setVoltage(voltage);
    }

    public void setMode(IntakeModes mode){
        if(!stateFulControl){
            intakeStates.setMode(mode);
            return;
        }
        
        if(mode.equals(INTAKE) && getCurrentState().equals(NOTE_IN_POS_IDLE)){
            intakeStates.setMode(STOP_INTAKE);
        }else if(mode.equals(SHOOT) && getCurrentState().equals(INTAKING)){
            intakeStates.setMode(STOP_INTAKE);
        }else{  
            intakeStates.setMode(mode);
        }
    }

    public void allowAutoMovement(){
        setVoltage(intakeStates.getDesiredVoltage());
    }

    public IntakeStates getCurrentState(){
        return intakeStates.getCurrentState();
    }

    public void stop(){
        m_intake.stopMotor();
    }

    public boolean isNoteIn(){
        return getCurrentState().equals(NOTE_IN_POS_IDLE);
    }

    public void setStateMachineOff(){
        stateFulControl = false;
    }

    public void setStateMachineOn(){
        stateFulControl = true;
    }

    /*---------------------------Commands---------------------------*/

    /**
     * This allows the Intake's state machine to have effect on the beheavior of the intake
     * This should be called every run loop cycle, set it as the default command 
     * @return Default Intake Command
     */
    public Command defaultIntakeCommand(){
        return Commands.sequence(
            runOnce(()->setMode(STOP_INTAKE)),
             run(()->allowAutoMovement()));
    }

    /**
     * Sets the mode of the Intake's state machine to "SHOOT" mode when scheduled,
     * it ends when non of the sensors detect a game piece(when the state is "SHOOTED")
     * @return Shoot Note Command
     */
    public Command shootNoteCommand(){
        return Commands.deadline(
            Commands.waitUntil(()->getCurrentState().equals(SHOOTED)), 
            Commands.runOnce(()->setMode(SHOOT)));
    }

    /**
     * Sets the mode of the Intake's state machine to "INTAKE" mode when scheduled
     * If is interrupted it sets the mode to "STOP_INTAKE"
     *  <-This is for TeleOp Only->
     * @return Intake Command
     */
    public Command intakeNoteCommand(){
        return Commands.startEnd(
            ()->setMode(INTAKE), ()->setMode(STOP_INTAKE)
        );
    }

    /**
     * Sets the mode of the Intake's state machine to "RELEASE" mode when scheduled
     * If is interrupted it sets the mode to "STOP_INTAKE"
     * <-This is for TeleOp Only->
     * @return Release Command
     */
    public Command releaseNoteCommand(){
        return Commands.startEnd(
            ()->setMode(RELEASE), ()->setMode(STOP_INTAKE)
        );
    }

    @Override
    public void periodic() {
        frontSensorResult = frontSensorDebouncer.calculate(!frontSensor.get());
        centerSensorResult = centerSensorDebouncer.calculate(!centerSensor.get());
        backSensorResult = backSensorDebouncer.calculate(!backSensor.get());
        backSensorLongResult = backSensorLongDebouncer.calculate(!backSensor.get());

        if(stateFulControl){
            intakeStates.updateSensors(
                ()->{return backSensorResult;}, 
                ()->{return frontSensorResult;}, 
                ()->{return centerSensorResult;});
            intakeStates.updateStates();
        }

        frontSensorPub.accept(frontSensorResult);
        centerSensorPub.accept(centerSensorResult);
        backSensorPub.accept(backSensorResult);
        backSensorLongPub.accept(backSensorLongResult);
        statesPub.accept(stateFulControl?getCurrentState().toString(): "No State Machine");
    }
}
