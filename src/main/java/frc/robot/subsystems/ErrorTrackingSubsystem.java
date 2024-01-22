// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class ErrorTrackingSubsystem extends SubsystemBase {

  ArrayList<CANSparkMax> motors = new ArrayList<>();
  ArrayList<StringPublisher> motorPublishers = new ArrayList<>();
  ArrayList<GenericEntry> motorEntries = new ArrayList<>();
  int currentMotor = 0;
  NetworkTable errorPublish;
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
    errorPublish = NetworkTableInstance.getDefault().getTable("CANSparkMax/Errors");
    errorsTab = Shuffleboard.getTab("CANSparkMax Errors");
    // Test code to register 25 can spark maxes
    for (int i = 0; i < 25; i++) {
      register(new CANSparkMax(i, MotorType.kBrushless));
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!motors.isEmpty()) {
      short faults = motors.get(currentMotor).getStickyFaults();
      motors.get(currentMotor).clearFaults();
      StringPublisher publisher = motorPublishers.get(currentMotor);
      String faultWords = faultWordToString(faults);
      publisher.set(faultWords);
      GenericEntry motorEntry = motorEntries.get(currentMotor);
      motorEntry.setBoolean(faultWords.isEmpty());
      if (currentMotor < motors.size() - 1 ) {
        currentMotor++;
      } else {
        currentMotor = 0;
      }
    }
  }

  // function to register can spark maxes
  public void register(CANSparkMax motor) {
    motorEntries.add(errorsTab
            .add(Integer.toString(motor.getDeviceId()), false)
            .withPosition((3 * (motors.size() % 3)), motors.size() / 3)
            // motor 1: (0, 0), 2: (3, 0), 3: (6, 0), 4: (9, 0), 5: (0, 1), 6: (3, 1), etc
            .withSize(3, 1).getEntry());
    motors.add(motor);
    motorPublishers.add(errorPublish.getStringTopic(Integer.toString(motor.getDeviceId())).publish());
  }

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
