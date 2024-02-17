package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.ProfiledPIDFFController;
import frc.robot.Config;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_arm; // bottom SparkMax motor controller
  private SparkAbsoluteEncoder m_absEncoder;
  private SparkPIDController m_pidControllerArm;

  ProfiledPIDFFController m_profiledFFController = new ProfiledPIDFFController();

  private ArmSubsystem() {
    m_arm = new CANSparkMax(Config.CANID.ARM_SPARK_CAN_ID, MotorType.kBrushless); // creates SparkMax motor controller
    m_arm.restoreFactoryDefaults();
    m_arm.setSmartCurrentLimit(Config.ArmConfig.CURRENT_LIMIT);
    m_arm.setInverted(Config.ArmConfig.SET_INVERTED); // sets movement direction
    m_arm.setIdleMode(IdleMode.kBrake); // sets brakes when there is no motion

    m_arm.setSoftLimit(SoftLimitDirection.kForward, Config.ArmConfig.arm_forward_limit);
    m_arm.setSoftLimit(SoftLimitDirection.kReverse, Config.ArmConfig.arm_reverse_limit);
    m_arm.enableSoftLimit(SoftLimitDirection.kForward, Config.ArmConfig.SOFT_LIMIT_ENABLE);
    m_arm.enableSoftLimit(SoftLimitDirection.kReverse, Config.ArmConfig.SOFT_LIMIT_ENABLE);

    // SparkMax periodic status frame 5: frequency absolute encoder position data is
    // sent over the can bus
    // SparkMax perioidc status frame 6: frequency absolute encoder velocity data is
    // sent over the can bus
    m_arm.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    m_arm.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    m_absEncoder = m_arm.getAbsoluteEncoder(Type.kDutyCycle);
    m_absEncoder.setInverted(Config.ArmConfig.INVERT_ENCODER);
    m_absEncoder.setPositionConversionFactor(Config.ArmConfig.armPositionConversionFactor);
    m_absEncoder.setVelocityConversionFactor(Config.ArmConfig.armVelocityConversionFactor);
    m_absEncoder.setZeroOffset(Math.toRadians(Config.ArmConfig.armAbsEncoderOffset));

    m_pidControllerArm = m_arm.getPIDController();
    m_pidControllerArm.setFeedbackDevice(m_absEncoder);

  }

  @Override
  public void periodic() {

  }

  // input angle_bottom in radians
  public void setJointAngle(double angle) {
    angle = MathUtil.clamp(angle, Math.toRadians(Config.ArmConfig.MIN_ARM_ANGLE_DEG), Math.toRadians(Config.ArmConfig.MAX_ARM_ANGLE_DEG));

    double targetPos = m_profiledFFController.getNextProfiledPIDPos(getPosition(), angle);
    m_pidControllerArm.setReference((targetPos), ControlType.kPosition, 0, calculateFF(angle));
  }

  // return radius
  public double getPosition() {
    return m_absEncoder.getPosition();
  }

  public void stopMotors() {
    m_arm.stopMotor();
  }

  private double calculateFF(double encoder1Rad) {
    double ArmMoment = Config.ArmConfig.ARM_FORCE * (Config.ArmConfig.LENGTH_ARM_TO_COG * Math.cos(encoder1Rad));
    return (ArmMoment) * 1 /* Apply the outputed voltage from the arm instead of 1 */;
  }
}
