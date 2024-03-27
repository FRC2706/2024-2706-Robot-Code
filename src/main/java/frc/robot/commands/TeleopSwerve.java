package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config;
import frc.robot.Config.Swerve;
import frc.robot.Config.Swerve.TeleopSpeeds;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopSwerve extends Command {
  private SwerveSubsystem s_Swerve;

  private CommandXboxController driver;
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private static TeleopSpeeds speed = TeleopSpeeds.MAX;
  private static boolean isFieldRelative = true;
  private static boolean isJoystickRelative = true;//-----------

  private static SlewRateLimiter translationLimiter = new SlewRateLimiter(6);
  private static SlewRateLimiter strafeLimiter = new SlewRateLimiter(6);
  private static SlewRateLimiter rotationLimiter = new SlewRateLimiter(8 * Math.PI);

  private double translationVal;
  private double strafeVal;
  private double rotationVal;

  private double prevHeading = 0.0;
  // private boolean controllingRotation = false;

  public TeleopSwerve(CommandXboxController driver) {
    this.s_Swerve = SwerveSubsystem.getInstance();
    addRequirements(s_Swerve);
    this.driver = driver;
  }

  public static void setSpeeds(TeleopSpeeds newSpeed) {
    speed = newSpeed;

    // Change acceleration limit and reset
    translationLimiter = new SlewRateLimiter(newSpeed.translationAccelLimit);
    strafeLimiter = new SlewRateLimiter(newSpeed.translationAccelLimit);
    rotationLimiter = new SlewRateLimiter(newSpeed.angularAccelLimit);
    resetAccelerationLimiters();
  }
  public static void setFieldRelative(boolean newIsFieldRelative) {
    isFieldRelative = newIsFieldRelative;
  }

  public static void resetAccelerationLimiters() {
    // translationLimiter.reset(MathUtil.clamp(SwerveSubsystem.getInstance().getFieldRelativeSpeeds().vxMetersPerSecond, -speed.translationalSpeed, speed.translationalSpeed));
    // strafeLimiter.reset(MathUtil.clamp(SwerveSubsystem.getInstance().getFieldRelativeSpeeds().vyMetersPerSecond, -speed.translationalSpeed, speed.translationalSpeed));
    // rotationLimiter.reset(MathUtil.clamp(SwerveSubsystem.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond, -speed.angularSpeed, speed.angularSpeed));
    translationLimiter.reset(SwerveSubsystem.getInstance().getFieldRelativeSpeeds().vxMetersPerSecond);
    strafeLimiter.reset(SwerveSubsystem.getInstance().getFieldRelativeSpeeds().vyMetersPerSecond);
    rotationLimiter.reset(SwerveSubsystem.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond);
  }

  @Override
  public void initialize() {
    resetAccelerationLimiters();
  }

  protected double calculateTranslationVal() {
    translationVal = MathUtil.applyDeadband(-driver.getRawAxis(translationAxis), Config.Swerve.stickDeadband)
        * speed.translationalSpeed;

    if(isJoystickRelative){//The joystick relative drive, in theory should work 
      if(driver.getRightX() > 0.2 && driver.getRightY() > 0.2){
        prevHeading = SwerveSubsystem.getInstance().calculateRotation(Rotation2d.fromRadians(Math.atan(driver.getRightY() / driver.getRightX())));
      }
      return prevHeading; 
    }
    else 
      return translationLimiter.calculate(translationVal);
  }

  protected double calculateStrafeVal() {
    strafeVal = MathUtil.applyDeadband(-driver.getRawAxis(strafeAxis), Config.Swerve.stickDeadband)
        * speed.translationalSpeed;
    return strafeLimiter.calculate(strafeVal);
  }

  protected double calculateRotationVal() {
    rotationVal = MathUtil.applyDeadband(-driver.getRawAxis(rotationAxis), Config.Swerve.stickDeadband)
        * speed.angularSpeed;

    return rotationLimiter.calculate(rotationVal);
  }

  // protected double calculateRotationVal() {
  //   rotationVal = MathUtil.applyDeadband(-driver.getRawAxis(rotationAxis), Config.Swerve.stickDeadband)
  //     * speed.angularSpeed;

  
  //   if(rotationVal != 0.0){
  //     getLastValue = false;
  //     return rotationVal = rotationLimiter.calculate(rotationVal);
  //     // return rotationLimiter.calculate(rotationVal);
  //   }else {
  //     rotationVal = rotationLimiter.calculate(rotationVal);

  //     if(!getLastValue){
  //       prevHeading = s_Swerve.getHeading();
  //       getLastValue = true;
  //     }
  //     if(getLastValue){
  //       return SwerveSubsystem.getInstance().calculateRotation(prevHeading);
  //     }else{
  //       return (rotationVal);
  //     }

  //   }
  //     //return(SwerveSubsystem.getInstance().calculateRotation(prevHeading));
  // }

  @Override
  public void execute() {

    s_Swerve.drive(
        new ChassisSpeeds(
            calculateTranslationVal(),
            calculateStrafeVal(),
            calculateRotationVal()),
            isFieldRelative,
        true);

    // if (rotationVal != 0.0 || Math.toDegrees(s_Swerve.getRobotRelativeSpeeds().omegaRadiansPerSecond) < 10) {
    //   prevHeading = s_Swerve.getHeading();
    // }
  }
}