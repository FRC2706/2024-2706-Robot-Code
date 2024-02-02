// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
     //TODO: check Team's 6328 2023 robots code and see what you can implement for constants, besides add the
  //Spark max burn manager
 public static final Mode currentMode = Mode.SIM;
 public static final boolean tuningMode = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

}
