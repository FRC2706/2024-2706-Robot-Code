// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class ErrorTrackingSubsystem extends SubsystemBase {

    ArrayList<CANSparkMax> motors = new ArrayList<>();
    ArrayList<StringPublisher> motorPublishers = new ArrayList<>();
    ArrayList<GenericEntry> statusTabEntries = new ArrayList<>();
    ArrayList<GenericEntry> errorsTabEntries = new ArrayList<>();
    int currentMotor = 0;
    NetworkTable errorPublish;
    ShuffleboardTab statusTab;
    ShuffleboardTab errorsTab;

    private static ErrorTrackingSubsystem instance;
    public static ErrorTrackingSubsystem getInstance(){
        if(instance == null){
            instance = new ErrorTrackingSubsystem();
        }
        return instance;
    }
    /** Creates a new ErrorTrackingSubsystem. */
    public ErrorTrackingSubsystem() {
        errorPublish = NetworkTableInstance.getDefault().getTable("CANSparkMax/Errors"); // Errors will be sent to NetworkTables
        statusTab = Shuffleboard.getTab("CANSparkMax Status"); // Status will be displayed as a boolean variable (whether the spark max is ok)
        errorsTab = Shuffleboard.getTab("CANSparkMax Errors");
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (!motors.isEmpty()) { // Ensure motor list is not empty!
            short faults = motors.get(currentMotor).getStickyFaults();
            motors.get(currentMotor).clearFaults(); // we need to make sure to clear the bus, so it doesn't just add up
            StringPublisher publisher = motorPublishers.get(currentMotor);
            String faultWords = faultWordToString(faults); // the faults are marked as IDs and must be converted to strings
            publisher.set(faultWords); // send to networktables
            GenericEntry statusEntry = statusTabEntries.get(currentMotor); // Gets status entry from shuffleboard
            statusEntry.setBoolean(faultWords.isEmpty());
            GenericEntry errorsEntry = errorsTabEntries.get(currentMotor); // Gets errors tab entry from shuffleboard
            errorsEntry.setString(faultWords);
            if (currentMotor < motors.size() - 1 ) {
                currentMotor++; // Next, we will check the next motor, and we will keep incrementing it to not crowd the bus.
            } else {
                currentMotor = 0;
            }
        }
    }

    /**
     * Function to register a new CANSparkMax to track errors from.
     * @param motor A CANSparkMax object (the motor).
     */
    public void register(CANSparkMax motor) {
        statusTabEntries.add(statusTab
                .add(Integer.toString(motor.getDeviceId()), false)
                .withPosition(( (motors.size() % 9)), motors.size() / 9)
                .withSize(1, 1).getEntry());
        errorsTabEntries.add(errorsTab
                .add(Integer.toString(motor.getDeviceId()), "")
                .withPosition(3 * (motors.size() % 3), motors.size() / 3)
                .withSize(3, 1).getEntry());
        motors.add(motor);
        motorPublishers.add(errorPublish.getStringTopic(Integer.toString(motor.getDeviceId())).publish());
    }

    /**
     * Function to convert a "fault word" to a string.
     * @param faults A short containing the faults.
     * @return A string containing the faults as a string.
     */
    public static String faultWordToString(short faults) {
        if (faults == 0) {
            return "";
        }

        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < 12; i++) {
            if (((1 << i) & (int) faults) != 0) {
                builder.append(CANSparkMax.FaultID.fromId(i).toString());
                builder.append(" ");
            }
        }
        return builder.toString();
    }

}

// TODO Log using a StringLogEntry
