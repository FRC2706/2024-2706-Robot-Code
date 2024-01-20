package frc.robot.subsystems;

import static frc.lib.lib2706.ErrorCheck.configureSpark;
import static frc.lib.lib2706.ErrorCheck.errSpark;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.lib.lib2706.UpdateSimpleFeedforward;
import frc.lib.lib3512.config.SwerveModuleConstants;
import frc.lib.lib3512.util.CANCoderUtil;
import frc.lib.lib3512.util.CANCoderUtil.CCUsage;
import frc.lib.lib3512.util.CANSparkMaxUtil;
import frc.lib.lib3512.util.CANSparkMaxUtil.Usage;
import frc.robot.Config;
import frc.robot.Robot;

public class SwerveModule {

  private NetworkTable swerveModuleTable;
  private DoublePublisher currentSpeedEntry;
  private DoublePublisher currentAngleEntry;
  private DoublePublisher speedError;
  private DoublePublisher angleError;
  private DoublePublisher desiredSpeedEntry;
  private DoublePublisher desiredAngleEntry;
  private DoubleEntry entryAngleOffset;
  private DoublePublisher canCoderAngleEntry;

  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

  private boolean synchronizeEncoderQueued = false;

  private SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
        Config.Swerve.driveKS, Config.Swerve.driveKV, Config.Swerve.driveKA);
  private UpdateSimpleFeedforward updateFeedforward;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, String ModuleName) {
    this.moduleNumber = moduleNumber;


    angleOffset = moduleConstants.angleOffset;

    String tableName = "SwerveChassis/SwerveModule" + ModuleName;
    swerveModuleTable = NetworkTableInstance.getDefault().getTable(tableName);
    updateFeedforward = new UpdateSimpleFeedforward((ff) -> feedforward = ff, swerveModuleTable, Config.Swerve.driveKS, Config.Swerve.driveKV, Config.Swerve.driveKA);
  
    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;

    desiredSpeedEntry = swerveModuleTable.getDoubleTopic("Desired speed (mps)").publish(PubSubOption.periodic(0.02));
    desiredAngleEntry = swerveModuleTable.getDoubleTopic("Desired angle (deg)").publish(PubSubOption.periodic(0.02));
    currentSpeedEntry = swerveModuleTable.getDoubleTopic("Current speed (mps)").publish(PubSubOption.periodic(0.02));
    currentAngleEntry = swerveModuleTable.getDoubleTopic("Current angle (rad)").publish(PubSubOption.periodic(0.02));
    speedError = swerveModuleTable.getDoubleTopic("Speed error (mps)").publish(PubSubOption.periodic(0.02));
    angleError = swerveModuleTable.getDoubleTopic("Angle error (deg)").publish(PubSubOption.periodic(0.02));
    entryAngleOffset = swerveModuleTable.getDoubleTopic("Angle Offset (deg)").getEntry(angleOffset.getDegrees());
    canCoderAngleEntry = swerveModuleTable.getDoubleTopic("Cancoder (deg)").publish(PubSubOption.periodic(0.02));
    
    entryAngleOffset.accept(angleOffset.getDegrees());

    resetToAbsolute();
    burnFlash();
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    setDesiredState(desiredState, isOpenLoop, false);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean isDisableAntiJitter) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not

    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    // desiredState.speedMetersPerSecond *= desiredState.angle.minus(getAngle()).getCos();
    
    setAngle(desiredState, isDisableAntiJitter);
    setSpeed(desiredState, isOpenLoop);


    desiredAngleEntry.accept(desiredState.angle.getRadians());
    desiredSpeedEntry.accept(desiredState.speedMetersPerSecond);
    speedError.accept((desiredState.speedMetersPerSecond)-(driveEncoder.getVelocity()));
    angleError.accept((desiredState.angle.getRadians())-(getAngle().getRadians()));
  }

  /**
   * Resets Position Encoder
   */
  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getRadians() - angleOffset.getRadians();
    integratedAngleEncoder.setPosition(absolutePosition);
    lastAngle = getAngle();
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    configureSpark("Angle restore factory defaults", () -> angleMotor.restoreFactoryDefaults());
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    configureSpark("Angle smart current limit", () -> angleMotor.setSmartCurrentLimit(Config.Swerve.angleContinuousCurrentLimit));
    angleMotor.setInverted(Config.Swerve.angleInvert);
    configureSpark("Angle idle mode", () -> angleMotor.setIdleMode(Config.Swerve.angleNeutralMode));
    configureSpark("Angle position conversion factor", () -> integratedAngleEncoder.setPositionConversionFactor(Config.Swerve.angleConversionFactor));
    configureSpark("Angle set P", () -> angleController.setP(Config.Swerve.angleKP));
    configureSpark("Angle set I", () -> angleController.setI(Config.Swerve.angleKI));
    configureSpark("Angle set D", () -> angleController.setD(Config.Swerve.angleKD));
    configureSpark("Angle set FF", () -> angleController.setFF(Config.Swerve.angleKFF));
    configureSpark("Angle set pid wrap min", () -> angleController.setPositionPIDWrappingMinInput(0));
    configureSpark("Angle set pid wrap max", () -> angleController.setPositionPIDWrappingMaxInput(2 * Math.PI));
    configureSpark("Angle set pid wrap", () -> angleController.setPositionPIDWrappingEnabled(true));
    configureSpark("Angle enable Volatage Compensation", () -> angleMotor.enableVoltageCompensation(Config.Swerve.voltageComp));
  }

  private void configDriveMotor() {
    configureSpark("Drive factory defaults", () -> driveMotor.restoreFactoryDefaults());
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    configureSpark("Drive smart current limit", () -> driveMotor.setSmartCurrentLimit(Config.Swerve.driveContinuousCurrentLimit));
    driveMotor.setInverted(Config.Swerve.driveInvert);
    configureSpark("Drive idle mode", () -> driveMotor.setIdleMode(Config.Swerve.driveNeutralMode));
    configureSpark("Drive velocity conversion factor", () -> driveEncoder.setVelocityConversionFactor(Config.Swerve.driveConversionVelocityFactor));
    configureSpark("Drive position conversion factor", () -> driveEncoder.setPositionConversionFactor(Config.Swerve.driveConversionPositionFactor));
    configureSpark("Drive set P", () -> driveController.setP(Config.Swerve.angleKP));
    configureSpark("Drive set I", () -> driveController.setI(Config.Swerve.angleKI));
    configureSpark("Drive set D", () -> driveController.setD(Config.Swerve.angleKD));
    configureSpark("Drive set FF", () -> driveController.setFF(Config.Swerve.angleKFF));
    configureSpark("Drive set pid wrap min", () -> driveController.setPositionPIDWrappingMinInput(0));
    configureSpark("Drive set pid wrap max", () -> driveController.setPositionPIDWrappingMaxInput(2 * Math.PI));
    configureSpark("Drive set pid wrap", () -> driveController.setPositionPIDWrappingEnabled(true));
    configureSpark("Drive voltage comp", () -> driveMotor.enableVoltageCompensation(Config.Swerve.voltageComp));
    configureSpark("Drive set position", () -> driveEncoder.setPosition(0.0));
  }

  /**
   * Save the configurations from flash to EEPROM.
   */
  private void burnFlash() {
    try {
      Thread.sleep(200);
    } 
    catch (Exception e) {}

    driveMotor.burnFlash();
    angleMotor.burnFlash();
  }

  /*
   * Sets Speed
   * 
   * @param desiredState
   * @param isOpenLoop Op
   */
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Config.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      errSpark("Drive set FF", 
        driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond)));
    }
  }

  /**
   * Sets Angle
   * 
   * @param desiredState 
   */
  private void setAngle(SwerveModuleState desiredState, boolean isDisableAntiJitter) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle;
    if (isDisableAntiJitter) {
      angle = desiredState.angle;

    // Anti jitter check
    } else if (Math.abs(desiredState.speedMetersPerSecond) <= (Config.Swerve.maxSpeed * 0.01)) {
      angle = lastAngle;

    } else {
      angle = desiredState.angle;
    }

    errSpark("Angle set reference", angleController.setReference(angle.getRadians(), CANSparkBase.ControlType.kPosition));
    lastAngle = angle;
  }

  /**
   * Returns Angle
   * 
   * @return Angle
   */
  private Rotation2d getAngle() {
    return (new Rotation2d(integratedAngleEncoder.getPosition()));
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  public void periodic() {
    //update network tables
    currentSpeedEntry.accept(driveEncoder.getVelocity());
    currentAngleEntry.accept(getAngle().getRadians());
    updateFeedforward.checkForUpdates();
    canCoderAngleEntry.accept(getCanCoder().getDegrees());

    if (Config.swerveTuning) {  
      angleOffset = Rotation2d.fromDegrees(entryAngleOffset.get());
    }
  }

  public boolean isModuleSynced(){
    if (Math.abs(getAngle().getDegrees() - (getCanCoder().getRadians() - angleOffset.getRadians())) < Config.Swerve.synchTolerance) {
      return true;
    }
    else{
      return false;
    }
  }

}
