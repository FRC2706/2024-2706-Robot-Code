// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.subsystems.IntakeStatesMachine.IntakeModes.*;
import static frc.robot.subsystems.IntakeStatesMachine.IntakeStates.*;
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
import frc.robot.subsystems.IntakeStatesMachine.IntakeModes;
import frc.robot.subsystems.IntakeStatesMachine.IntakeStates;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase{
    private CANSparkMax m_intake;
    private boolean stateFulControl = false;
    private IntakeStatesMachine intakeStates = new IntakeStatesMachine();

    private DigitalInput sensor7;//Back
    private DigitalInput sensor8;//Center
    private DigitalInput sensor9;//Front

    private Debouncer sensor7Debouncer;
    private Debouncer sensor8Debouncer;
    private Debouncer sensor9Debouncer;

    private BooleanPublisher sensor7Pub;
    private BooleanPublisher sensor8Pub;
    private BooleanPublisher sensor9Pub;
    private StringPublisher statesPub;

    private boolean sensor7Result;
    private boolean sensor8Result;
    private boolean sensor9Result;

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

        sensor7 = new DigitalInput(7);
        sensor8 = new DigitalInput(8);
        sensor9 = new DigitalInput(9);

        sensor7Debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        sensor8Debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        sensor9Debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

        NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Intake");
        statesPub = intakeTable.getStringTopic("Intake's Current State").publish(PubSubOption.periodic(0.02));
        sensor7Pub = intakeTable.getBooleanTopic("sensor 7 result").publish(PubSubOption.periodic(0.02));
        sensor8Pub = intakeTable.getBooleanTopic("sensor 8 result").publish(PubSubOption.periodic(0.02));
        sensor9Pub = intakeTable.getBooleanTopic("sensor 9 result").publish(PubSubOption.periodic(0.02));
    }

    public boolean isSensor7True() {
        return sensor7Result;
    }

    public boolean isSensor8True() {
        return sensor8Result;
    }

    public boolean isSensor9True() {
        return sensor9Result;
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
        sensor7Result = sensor7Debouncer.calculate(!sensor7.get());
        sensor8Result = sensor8Debouncer.calculate(!sensor8.get());
        sensor9Result = sensor9Debouncer.calculate(!sensor9.get());

        if(stateFulControl){
            intakeStates.updateSensors(
                ()->{return sensor7Result;}, //Back sensor
                ()->{return sensor9Result;}, //Front sensor
                ()->{return sensor8Result;});//Center sensor (Could be removable)
            intakeStates.updateStates();
        }

        sensor7Pub.accept(sensor7Result);
        sensor8Pub.accept(sensor8Result);
        sensor9Pub.accept(sensor9Result);
        statesPub.accept(stateFulControl?getCurrentState().toString(): "No State Machine");
    }
}
