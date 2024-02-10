// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BlingSubsystem;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class BlingCommand extends InstantCommand {
  public enum BlingColour {
    DISABLED,
    PURPLE,
    BLUE,
    RED,
    YELLOW,
    HONEYDEW,
    RAINBOW,
    FIRE,
    RGBFADE,
    WHITESTROBE,
    REDSTROBE,
    YELLOWSTROBE,
    BLUESTROBE,
    PURPLESTROBE,
  }
  public BlingSubsystem bling = BlingSubsystem.getINSTANCE();
  public BlingColour blingColour;

  public BlingCommand(BlingColour colour) {
    blingColour = colour;

    // Use addRequirements() here to declare subsystem dependencies.
    if( bling != null )
      addRequirements(bling);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (bling != null)
    {
      if (blingColour != null) {
        bling.setBrightness();
      }
      switch( blingColour )
      {
        case DISABLED:
          bling.setDisabled();
          break;
        case PURPLE:
          bling.setPurple();
          break;
        case BLUE:
          bling.setBlue();
          break;
        case RED:
          bling.setRed();
          break;
        case YELLOW:
          bling.setYellow();
          break;
        case HONEYDEW:
          bling.setHoneydew();
          break;
        case RAINBOW: 
          setRainbow();
          break;
        case FIRE: 
          setFire();
          break;
        case RGBFADE:
          setRgbFade();
          break;
        case WHITESTROBE:
          setWhiteStrobe();
          break;
        case REDSTROBE:
          setRedStrobe();
          break;
        case YELLOWSTROBE:
          setYellowStrobe();
          break;
        case BLUESTROBE:
          setBlueStrobe();
          break;
        case PURPLESTROBE:
          setPurpleStrobe();
          break;
        default:
          break;
      }
    }

    }
  
  public void setRainbow() {
     //* @param brightness The brightness of the LEDs 1
     //* @param speed How fast the rainbow travels through the leds 0.01
     //* @param numLed How many LEDs are controlled by the CANdle 64
    RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.01, 64);

    bling.setAnimation(rainbowAnim);

  }

  public void setFire()
  {
     //* @param brightness How bright should the animation be 1
     //* @param speed How fast will the flame be processed at 0.000001
     //* @param numLed How many LEDs is the CANdle controlling 64
     //* @param sparking The rate at which the Fire "Sparks" 0.8
     //* @param cooling The rate at which the Fire "Cools" along the travel 0.4
    FireAnimation fireAnimation = new FireAnimation(1, 0.000001, 64, 0.8, 0.4);

    bling.setAnimation(fireAnimation);
  }

  public void setWhiteStrobe()
  {
   //  * @param r How much red should the color have 255
   //  * @param g How much green should the color have 255
   //  * @param b How much blue should the color have 255
   //  * @param w How much white should the color have 255
   //  * @param speed How fast should the color travel the strip 0.001
   //  * @param numLed How many LEDs the CANdle controls 64
    StrobeAnimation strobeAnimation = new StrobeAnimation(255, 255, 255, 255, 0.8, 64);

    bling.setAnimation(strobeAnimation);
  }

  public void setPurpleStrobe()
  {
   //  * @param r How much red should the color have 138
   //  * @param g How much green should the color have 43
   //  * @param b How much blue should the color have 226
   //  * @param w How much white should the color have 127
   //  * @param speed How fast should the color travel the strip 0.001
   //  * @param numLed How many LEDs the CANdle controls 64
    StrobeAnimation strobeAnimation = new StrobeAnimation(138, 43, 226, 127, 0.001, 64); //TODO: test all of the rgbw bling values

    bling.setAnimation(strobeAnimation);
  }

  public void setRedStrobe()
  {
   //  * @param r How much red should the color have 255
   //  * @param g How much green should the color have 0
   //  * @param b How much blue should the color have 0
   //  * @param w How much white should the color have 127
   //  * @param speed How fast should the color travel the strip 0.001
   //  * @param numLed How many LEDs the CANdle controls 64
    StrobeAnimation strobeAnimation = new StrobeAnimation(255, 0, 0, 127, 0.001, 64);

    bling.setAnimation(strobeAnimation);
  }

  public void setBlueStrobe()
  {
   //  * @param r How much red should the color have 0
   //  * @param g How much green should the color have 0
   //  * @param b How much blue should the color have 255
   //  * @param w How much white should the color have 127
   //  * @param speed How fast should the color travel the strip 0.001
   //  * @param numLed How many LEDs the CANdle controls 64
    StrobeAnimation strobeAnimation = new StrobeAnimation(0, 0, 255, 127, 0.001, 64);

    bling.setAnimation(strobeAnimation);
  }

  public void setYellowStrobe()
  {
   //  * @param r How much red should the color have 255
   //  * @param g How much green should the color have 255
   //  * @param b How much blue should the color have 0
   //  * @param w How much white should the color have 127
   //  * @param speed How fast should the color travel the strip 0.001
   //  * @param numLed How many LEDs the CANdle controls 64
    StrobeAnimation strobeAnimation = new StrobeAnimation(255, 255, 0, 127, 0.001, 64);

    bling.setAnimation(strobeAnimation);
  }

  public void setRgbFade()
  {
     //* @param brightness How bright the LEDs are 0.7
     //* @param speed How fast the LEDs fade between Red, Green, and Blue 0.1
     //* @param numLed How many LEDs are controlled by the CANdle 64
    RgbFadeAnimation rgbFadeAnimation = new RgbFadeAnimation(0.7, 0.1, 64);

    bling.setAnimation(rgbFadeAnimation);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
