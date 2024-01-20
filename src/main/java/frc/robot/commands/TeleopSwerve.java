package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config;
import frc.robot.Config.Swerve.TeleopSpeeds;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopSwerve extends Command {
  private SwerveSubsystem s_Swerve;

  private CommandXboxController driver;
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  TeleopSpeeds speeds;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private double translationVal;
  private double strafeVal;
  private double rotationVal;

  public TeleopSwerve(
      SwerveSubsystem s_Swerve,
      CommandXboxController driver,
      TeleopSpeeds speeds) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.driver = driver;
    this.speeds = speeds;
  }

  @Override
  public void execute() {
    /* Get Values and apply deadband to limit unwanted movement*/
    translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(-driver.getRawAxis(translationAxis), Config.Swerve.stickDeadband));
    strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(-driver.getRawAxis(strafeAxis), Config.Swerve.stickDeadband));
    rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(-driver.getRawAxis(rotationAxis), Config.Swerve.stickDeadband));

    s_Swerve.drive(
        new ChassisSpeeds(
          translationVal * speeds.translationalSpeed, 
          strafeVal * speeds.translationalSpeed,
          rotationVal * speeds.angularSpeed),
        true,
        true);
  }
}