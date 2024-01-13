// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//imports
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//class
public class LimeLightSubsystem extends SubsystemBase {

  //constants
  private long[] sendData = {10,100,29,36,63,21,892,345};
  
  private String networkTableName = "limelight";

  //declarations
  private static LimeLightSubsystem instance;
  private IntegerArrayPublisher pubData;
  
  public static LimeLightSubsystem getInstance(){
    if (instance == null){
      instance = new LimeLightSubsystem();
    }
    return instance;
  }

  /** Creates a new photonAprilTag. */
  private LimeLightSubsystem() {
    //networktable publishers
    pubData = NetworkTableInstance.getDefault().getTable(networkTableName).getIntegerArrayTopic("llpython").publish(PubSubOption.periodic(0.02));
    System.out.println("starting");
  }

  @Override
  public void periodic() {
    //publish to networktables
    pubData.accept(sendData);
    System.out.println("running");
  
}
}
