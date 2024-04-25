package frc.robot.subsystems;

import static frc.lib.lib2706.ErrorCheck.configureSpark;
import static frc.lib.lib2706.ErrorCheck.errSpark;

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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.ProfiledPIDFFController;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;
import frc.robot.Config.ArmConfig;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance = null; // static object that contains all movement controls

  private static final MotorType motorType = MotorType.kBrushless; // defines brushless motortype
  private final CANSparkMax m_arm; // bottom SparkMax motor controller

  // network table entry
  private final String m_tuningTable = "Arm/ArmTuning";
  private final String m_dataTable = "Arm/ArmData";

  // network table entries
  private DoubleEntry m_armPSubs;
  private DoubleEntry m_armISubs;
  private DoubleEntry m_armDSubs;
  private DoubleEntry m_armIzSubs;
  private DoubleEntry m_armFFSubs;
  private DoublePublisher m_armSetpointPub;
  private DoublePublisher m_armVelPub;
  private DoublePublisher m_armFFTestingVolts;
  private DoubleEntry m_armOffset;
  private DoublePublisher m_targetAngle;
  private DoublePublisher m_armPosPub;

  // for arm ff
  private DoubleEntry m_armMomentToVoltage;

  //spark absolute encoder
  private SparkAbsoluteEncoder m_absEncoder;  
  //embedded relative encoder
  private SparkPIDController m_pidControllerArm;    

  private final TrapezoidProfile.Constraints m_constraints = 
    new TrapezoidProfile.Constraints(Config.ArmConfig.MAX_VEL, Config.ArmConfig.MAX_ACCEL);
  private final ProfiledPIDController m_ProfiledPIDController = 
    new ProfiledPIDController(1.6,0.002,40, m_constraints, 0.02);


  public static ArmSubsystem getInstance() {
    if (instance == null) {
      SubsystemChecker.subsystemConstructed(SubsystemType.ArmSubsystem);
      instance = new ArmSubsystem();
    }
    return instance;
  }

  private ArmSubsystem() {
    m_arm = new CANSparkMax(Config.ArmConfig.ARM_SPARK_CAN_ID, motorType); // creates SparkMax motor controller
    configureSpark("Arm restore factory defaults", () -> (m_arm.restoreFactoryDefaults()));
    configureSpark("arm set CANTimeout", () -> m_arm.setCANTimeout(Config.CANTIMEOUT_MS));
    configureSpark("Arm set current limits", () -> m_arm.setSmartCurrentLimit(Config.ArmConfig.CURRENT_LIMIT));
    m_arm.setInverted(Config.ArmConfig.SET_INVERTED); // sets movement direction
    configureSpark("Arm set brakes when idle", () -> (m_arm.setIdleMode(IdleMode.kBrake))); // sets brakes when there is  no motion
    configureSpark("Arm voltage compesentation", () -> m_arm.enableVoltageCompensation(6));                                                                                           

    configureSpark("Arm set soft limits forward",
        () -> (m_arm.setSoftLimit(SoftLimitDirection.kForward, (float) (Config.ArmConfig.arm_forward_limit))));
    configureSpark("Arm sets soft limits reverse",
        () -> (m_arm.setSoftLimit(SoftLimitDirection.kReverse, (float) (Config.ArmConfig.arm_reverse_limit))));
    configureSpark("Arm enables soft limits forward",
        () -> (m_arm.enableSoftLimit(SoftLimitDirection.kForward, Config.ArmConfig.SOFT_LIMIT_ENABLE)));
    configureSpark("Arm enable soft limit reverse",
        () -> (m_arm.enableSoftLimit(SoftLimitDirection.kReverse, Config.ArmConfig.SOFT_LIMIT_ENABLE)));

    // SparkMax periodic status frame 5: frequency absolute encoder position data is
    // sent over the can bus
    // SparkMax perioidc status frame 6: frequency absolute encoder velocity data is
    // sent over the can bus
    configureSpark("Arm set periodic frame period", () -> m_arm.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20));
    configureSpark("Arm set periodic frame period", () -> m_arm.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20));

    m_absEncoder = m_arm.getAbsoluteEncoder(Type.kDutyCycle);
    configureSpark("Absolute encoder set inerted", () -> m_absEncoder.setInverted(Config.ArmConfig.INVERT_ENCODER));
    configureSpark("Absolute encoder set position conersation factor",
        () -> m_absEncoder.setPositionConversionFactor(Config.ArmConfig.armPositionConversionFactor));
    configureSpark("Absolute Encoder set velocity conversion factor",
        () -> m_absEncoder.setVelocityConversionFactor(Config.ArmConfig.armVelocityConversionFactor));
    configureSpark("Absolute encoder set zero offset",
        () -> m_absEncoder.setZeroOffset(Math.toRadians(Config.ArmConfig.armAbsEncoderOffset)));

    m_pidControllerArm = m_arm.getPIDController();
    configureSpark("Pid controller arm set feedback device", () -> m_pidControllerArm.setFeedbackDevice(m_absEncoder));

    NetworkTable ArmTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTable);
    m_armPSubs = ArmTuningTable.getDoubleTopic("P").getEntry(Config.ArmConfig.arm_kP);
    m_armISubs = ArmTuningTable.getDoubleTopic("I").getEntry(Config.ArmConfig.arm_kI);
    m_armDSubs = ArmTuningTable.getDoubleTopic("D").getEntry(Config.ArmConfig.arm_kD);
    m_armIzSubs = ArmTuningTable.getDoubleTopic("IZone").getEntry(Config.ArmConfig.arm_kIz);
    m_armFFSubs = ArmTuningTable.getDoubleTopic("FF").getEntry(Config.ArmConfig.arm_kFF);
    // m_topArmOffset =
    // topArmTuningTable.getDoubleTopic("Offset").getEntry(ArmConfig.top_arm_offset);
    m_armMomentToVoltage = ArmTuningTable.getDoubleTopic("MomentToVoltage")
        .getEntry(Config.ArmConfig.MOMENT_TO_VOLTAGE);

    m_armFFSubs.setDefault(Config.ArmConfig.arm_kFF);
    m_armPSubs.setDefault(Config.ArmConfig.arm_kP);
    m_armISubs.setDefault(Config.ArmConfig.arm_kI);
    m_armDSubs.setDefault(Config.ArmConfig.arm_kD);
    m_armIzSubs.setDefault(Config.ArmConfig.arm_kIz);

    NetworkTable ArmDataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);

    m_armPosPub = ArmDataTable.getDoubleTopic("MeasuredAngleDeg").publish(PubSubOption.periodic(0.02));
    m_armVelPub = ArmDataTable.getDoubleTopic("MeasuredVelocity").publish(PubSubOption.periodic(0.02));
    m_armFFTestingVolts= ArmDataTable.getDoubleTopic("FFTestingVolts").publish(PubSubOption.periodic(0.02));
    m_targetAngle = ArmDataTable.getDoubleTopic("TargetAngleDeg").publish(PubSubOption.periodic(0.02));

    updatePID0Settings();
    updatePID1Settings();

    burnFlash();
    configureSpark("Arm set CANTimeout", () -> m_arm.setCANTimeout(0));

    ErrorTrackingSubsystem.getInstance().register(m_arm);
  }

  public void updatePID0Settings() {
    configureSpark("Arm set FF", () -> (m_pidControllerArm.setFF(m_armFFSubs.get(), 0)));
    configureSpark("Arm set P", () -> (m_pidControllerArm.setP(m_armPSubs.get(), 0)));
    configureSpark("Arm set I", () -> (m_pidControllerArm.setI(m_armISubs.get(), 0)));
    configureSpark("Arm set D", () -> (m_pidControllerArm.setD(m_armDSubs.get(), 0)));
    configureSpark("Arm set Iz", () -> (m_pidControllerArm.setIZone(m_armIzSubs.get(), 0)));
    configureSpark("Arm set Output Range",
        () -> (m_pidControllerArm.setOutputRange(Config.ArmConfig.min_output, Config.ArmConfig.max_output)));
  }

  public void updatePID1Settings() {
    configureSpark("Arm set far FF", () -> m_pidControllerArm.setFF(ArmConfig.arm_far_kFF, 1));
    configureSpark("Arm set far P", () -> m_pidControllerArm.setP(ArmConfig.arm_far_kP, 1));
    configureSpark("Arm set far I", () -> m_pidControllerArm.setI(ArmConfig.arm_far_kI, 1));
    configureSpark("Arm set far D", () -> m_pidControllerArm.setD(ArmConfig.arm_far_kD, 1));
    configureSpark("Arm set far Iz", () -> m_pidControllerArm.setIZone(ArmConfig.arm_far_iZone, 1));
  }

  @Override
  public void periodic() {
    m_armPosPub.accept(Math.toDegrees(getPosition()));
    m_armVelPub.accept(Math.toDegrees(m_absEncoder.getVelocity()));
  }

    // input angle_bottom in radians(
  public void setJointAngle(double angle) {
    double clampedAngle = MathUtil.clamp(angle, Math.toRadians(Config.ArmConfig.MIN_ARM_ANGLE_DEG),
        Math.toRadians(Config.ArmConfig.MAX_ARM_ANGLE_DEG));

    // pidSlot 1 is tuned well for setpoints between 25 deg and 45 deg
    double angleDeg = Math.toDegrees(angle);
    int pidSlot = 0;
    if (angleDeg < 25) {
      pidSlot = 0;
    } else if (angleDeg >= 25 && angleDeg < 55) {
      pidSlot = 1;
    } else if (angleDeg >= 55) {
      pidSlot = 0;
    }

    m_ProfiledPIDController.calculate(getPosition(), clampedAngle);
    double targetPos = m_ProfiledPIDController.getSetpoint().position;

    //m_pidControllerArm.setReference((targetPos), ControlType.kPosition, 0, calculateFF(clampedAngle));
    m_pidControllerArm.setReference(targetPos + Math.toRadians(ArmConfig.shiftEncoderRange), ControlType.kPosition, pidSlot, 0);

     m_targetAngle.accept(Math.toDegrees(targetPos));
  }

  public void resetProfiledPIDController() {
     m_ProfiledPIDController.reset(getPosition(), m_absEncoder.getVelocity());
  }


  
    //return radius
    public double getPosition() {
      return m_absEncoder.getPosition() - Math.toRadians(ArmConfig.shiftEncoderRange);
    }
  
    public void stopMotors() {
      m_arm.stopMotor();
    }

    public void burnFlash() {
      try {
        Thread.sleep(200);
      } 
      catch (Exception e) {}

      errSpark("Arm burn flash", m_arm.burnFlash());      
    }

    private double calculateFF(double encoder1Rad) {
      //double ArmMoment = Config.ArmConfig.ARM_FORCE * (Config.ArmConfig.LENGTH_ARM_TO_COG*Math.cos(encoder1Rad));
      //return (ArmMoment) * m_armMomentToVoltage.get();

      double toTunedConst = m_armMomentToVoltage.get();
      return toTunedConst*Math.cos(encoder1Rad);
    }

    public void isAtSetpoint() {
    }

    public void setArmIdleMode(IdleMode mode) {
      m_arm.setIdleMode(mode);
    }

    public void testFeedForward(double additionalVoltage) {
      double voltage = additionalVoltage + calculateFF(getPosition());
      m_pidControllerArm.setReference(voltage, ControlType.kVoltage);
      m_armFFTestingVolts.accept(voltage);
    }

}
