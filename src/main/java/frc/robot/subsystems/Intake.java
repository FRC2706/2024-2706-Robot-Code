// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.commands.AutoIntake;
import frc.robot.subsystems.IntakeStates.Modes;

/** Add your docs here. */
public class Intake extends SubsystemBase{
    private CANSparkMax m_intake;
    private IntakeStates intakeStates = new IntakeStates();

    //TODO: For sake of god, we hae to label them
    private DigitalInput sensor7; //DigitInput instance
    private DigitalInput sensor8; //
    private DigitalInput sensor9;

    private Debouncer sensor7Debouncer;
    private Debouncer sensor8Debouncer;
    private Debouncer sensor9Debouncer;

    private BooleanPublisher sensor7Pub;
    private BooleanPublisher sensor8Pub;
    private BooleanPublisher sensor9Pub;
    
    private boolean sensor7Result;
    private boolean sensor8Result;
    private boolean sensor9Result;

    private static Intake instance;
    public static Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }

    private Intake() {
        System.out.println("[Init]Creating Intake");
        m_intake = new CANSparkMax(Config.Intake.INTAKE, MotorType.kBrushless);
        m_intake.restoreFactoryDefaults();
        m_intake.setInverted(true);
        m_intake.setSmartCurrentLimit(70);
        m_intake.setIdleMode(IdleMode.kBrake);

        sensor7 = new DigitalInput(7);
        sensor8 = new DigitalInput(8);
        sensor9 = new DigitalInput(9);

        sensor7Debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        sensor8Debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        sensor9Debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

        NetworkTable sensorTable = NetworkTableInstance.getDefault().getTable("sensors");

        sensor7Pub = sensorTable.getBooleanTopic("sensor 7 result").publish(PubSubOption.periodic(0.02));
        sensor8Pub = sensorTable.getBooleanTopic("sensor 8 result").publish(PubSubOption.periodic(0.02));
        sensor9Pub = sensorTable.getBooleanTopic("sensor 9 result").publish(PubSubOption.periodic(0.02));
        
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

    public void setSpeed(Double speed) {
        m_intake.set(speed);
    }

    public void setVoltage(double voltage){
        m_intake.setVoltage(voltage);
    }

    public void setMode(Modes mode){
        intakeStates.setMode(mode);
    }

    public void allowAutoMovement(){
        setVoltage(intakeStates.getDesiredVoltage());
    }

    @Override
    public void periodic() {
        sensor7Result = sensor7Debouncer.calculate(!sensor7.get());
        sensor8Result = sensor8Debouncer.calculate(!sensor8.get());
        sensor9Result = sensor9Debouncer.calculate(!sensor9.get());

        sensor7Pub.accept(sensor7Result);
        sensor8Pub.accept(sensor8Result);
        sensor9Pub.accept(sensor9Result);

        //Correct them and put them in the right order
        intakeStates.updateSensors(()->{return sensor7Result;},()->{return sensor9Result;}, ()->{return sensor8Result;});
        intakeStates.updateStates();
        SmartDashboard.putString("states", intakeStates.getCurrentState().toString());
    }

    public void stop(){
        m_intake.stopMotor();
    }
}
