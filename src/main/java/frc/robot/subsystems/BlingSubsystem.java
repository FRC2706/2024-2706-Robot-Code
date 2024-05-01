/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class BlingSubsystem extends SubsystemBase {

  private CANdle candle;
  public final double Brightness = 0.5;
  private static BlingSubsystem INSTANCE = null;

  /**
   * Creates a new Bling.
   */
  private BlingSubsystem() {
    if (Config.BlingConstants.CANDLE != -1) {
      candle = new CANdle(Config.BlingConstants.CANDLE);

      CANdleConfiguration config = new CANdleConfiguration();
      config.stripType = LEDStripType.RGB; // set the strip type to RGB
      config.brightnessScalar = Brightness; // dim the LEDs to half brightness

      candle.configAllSettings(config);
    } else {
      candle = null;
    }
  }

  public static BlingSubsystem getINSTANCE() {
    if (Config.BlingConstants.CANDLE == -1) {
      INSTANCE = null;
    } else if (INSTANCE == null) {
      INSTANCE = new BlingSubsystem();
    }

    return INSTANCE;
  }


  public void setBrightness() {
    candle.configBrightnessScalar(Brightness);
  }

  public void setDisabled() {
    candle.configBrightnessScalar(0.0);
    candle.clearAnimation(0);
  }

  public void setOrange() {
    candle.clearAnimation(0);
    candle.setLEDs(245, 141, 66);
  }

  public void setPurple() {
    candle.clearAnimation(0);
    candle.setLEDs(138, 43, 226);
  }

  public void setBlue() {
    candle.clearAnimation(0);
    candle.setLEDs(0, 0, 255);
  }

  public void setRed() {
    candle.clearAnimation(0);
    candle.setLEDs(255, 0, 0);
  }

  public void setHoneydew() {
    candle.clearAnimation(0);
    candle.setLEDs(240, 255, 240);
  }

  public void setYellow() {
    candle.clearAnimation(0);
    candle.setLEDs(255, 255, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setAnimation(Animation animation) {

    candle.animate(animation);
  }

}