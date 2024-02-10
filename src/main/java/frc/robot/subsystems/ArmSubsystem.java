package frc.robot.subsystems;

import static frc.lib.lib2706.ErrorCheck.configureSpark;

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

public class ArmSubsystem extends SubsystemBase{
     private static ArmSubsystem instance = null; //static object that contains all movement controls

     private static final MotorType motorType = MotorType.kBrushless; //defines brushless motortype
     private final CANSparkMax m_arm; //bottom SparkMax motor controller
     
     //network table entry
     private final String m_tuningTable = "Arm/ArmTuning";
     private final String m_dataTable = "Arm/ArmData";
  
     //network table entries 
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

    //for  arm ff
    private DoubleEntry m_armMomentToVoltage;

    //spark absolute encoder
    private SparkAbsoluteEncoder m_absEncoder;  
    //embedded relative encoder
    //private RelativeEncoder m_Encoder;
    private SparkPIDController m_pidControllerArm;  

    ProfiledPIDFFController m_profiledFFController = new ProfiledPIDFFController();

    public static ArmSubsystem getInstance() {
      if (instance == null) {
        SubsystemChecker.subsystemConstructed(SubsystemType.ArmSubsystem);
        instance = new ArmSubsystem();
      }
      return instance;
    }
    private ArmSubsystem() {
      m_arm = new CANSparkMax(Config.CANID.ARM_SPARK_CAN_ID, motorType); //creates SparkMax motor controller 
      configureSpark("Arm restore factory defaults", () -> (m_arm.restoreFactoryDefaults()));
      m_arm.setCANTimeout(100);
      configureSpark("Arm set current limits", () -> m_arm.setSmartCurrentLimit(Config.ArmConfig.CURRENT_LIMIT));
      m_arm.setInverted(Config.ArmConfig.SET_INVERTED); //sets movement direction
      configureSpark("Arm set brakes when idle", () -> (m_arm.setIdleMode(IdleMode.kBrake))); //sets brakes when there is no motion
      
      configureSpark("Arm set soft limits forward", () -> (m_arm.setSoftLimit(SoftLimitDirection.kForward, Config.ArmConfig.arm_forward_limit)));
      configureSpark("Arm sets soft limits reverse", () -> (m_arm.setSoftLimit(SoftLimitDirection.kReverse, Config.ArmConfig.arm_reverse_limit)));
      configureSpark("Arm enables soft limits forward", () ->(m_arm.enableSoftLimit(SoftLimitDirection.kForward, Config.ArmConfig.SOFT_LIMIT_ENABLE)));
      configureSpark("Arm enable soft limit reverse", () -> (m_arm.enableSoftLimit(SoftLimitDirection.kReverse, Config.ArmConfig.SOFT_LIMIT_ENABLE)));

      //SparkMax periodic status frame 5: frequency absolute encoder position data is sent over the can bus
      //SparkMax perioidc status frame 6: frequency absolute encoder velocity data is sent over the can bus
      m_arm.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
      m_arm.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

      m_absEncoder = m_arm.getAbsoluteEncoder(Type.kDutyCycle);
      m_absEncoder.setInverted(Config.ArmConfig.INVERT_ENCODER);
      m_absEncoder.setPositionConversionFactor(Config.ArmConfig.armPositionConversionFactor);
      m_absEncoder.setVelocityConversionFactor(Config.ArmConfig.armVelocityConversionFactor);
      m_absEncoder.setZeroOffset(Math.toRadians(Config.ArmConfig.armAbsEncoderOffset));

      m_pidControllerArm = m_arm.getPIDController();
      m_pidControllerArm.setFeedbackDevice(m_absEncoder);

      
    NetworkTable ArmTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTable);
    m_armPSubs = ArmTuningTable.getDoubleTopic("P").getEntry(Config.ArmConfig.arm_kP);
    m_armISubs = ArmTuningTable.getDoubleTopic("I").getEntry(Config.ArmConfig.arm_kI);
    m_armDSubs = ArmTuningTable.getDoubleTopic("D").getEntry(Config.ArmConfig.arm_kD);
    m_armIzSubs = ArmTuningTable.getDoubleTopic("IZone").getEntry(Config.ArmConfig.arm_kIz);
    m_armFFSubs = ArmTuningTable.getDoubleTopic("FF").getEntry(Config.ArmConfig.arm_kFF);
   //m_topArmOffset = topArmTuningTable.getDoubleTopic("Offset").getEntry(ArmConfig.top_arm_offset);
    m_armMomentToVoltage = ArmTuningTable.getDoubleTopic("MomentToVoltage").getEntry(Config.ArmConfig.MOMENT_TO_VOLTAGE);

    m_armFFSubs.setDefault(Config.ArmConfig.arm_kFF);
    m_armPSubs.setDefault(Config.ArmConfig.arm_kP);
    m_armISubs.setDefault(Config.ArmConfig.arm_kI);
    m_armDSubs.setDefault(Config.ArmConfig.arm_kD);
    m_armIzSubs.setDefault(Config.ArmConfig.arm_kIz);

    NetworkTable ArmDataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);
    
    m_armPosPub = ArmDataTable.getDoubleTopic("MeasuredAngleDeg").publish(PubSubOption.periodic(0.02));
    m_targetAngle = ArmDataTable.getDoubleTopic("TargetAngleDeg").publish(PubSubOption.periodic(0.02));

    updatePIDSettings();
    m_arm.setCANTimeout(0);

  }
  public void updatePIDSettings() {
    configureSpark("Arm set FF", () ->(m_pidControllerArm.setFF(m_armFFSubs.get())));
    configureSpark("Arm set P", () -> (m_pidControllerArm.setP(m_armPSubs.get())));
    configureSpark("Arm set I", () ->(m_pidControllerArm.setI(m_armISubs.get())));
    configureSpark("Arm set D", () -> (m_pidControllerArm.setD(m_armDSubs.get())));
    configureSpark("Arm set Iz", () -> (m_pidControllerArm.setIZone(m_armIzSubs.get())));
    configureSpark("Arm set Output Range", () -> (m_pidControllerArm.setOutputRange(Config.ArmConfig.min_output, Config.ArmConfig.max_output)));
  }
  @Override
  public void periodic() {
    m_armPosPub.accept(Math.toDegrees(m_absEncoder.getPosition()));
    m_armVelPub.accept(Math.toDegrees(m_absEncoder.getVelocity()));
  }
    //input angle_bottom in radians(
    public void setJointAngle(double angle) {
      MathUtil.clamp(angle, Math.toRadians(Config.ArmConfig.MIN_ARM_ANGLE_DEG), Math.toRadians(Config.ArmConfig.MAX_ARM_ANGLE_DEG));
    
      double targetPos = m_profiledFFController.getNextProfiledPIDPos(getPosition(), angle);
      m_pidControllerArm.setReference((targetPos), ControlType.kPosition, 0, calculateFF(angle));
      m_targetAngle.accept(Math.toDegrees(targetPos));
    }

  
    //return radius
    public double getPosition() {
      return m_absEncoder.getPosition();
    }
  
    public void stopMotors() {
      m_arm.stopMotor();
    }
    public void burnFlash() {
      configureSpark("Arm burn flash", () -> (m_arm.burnFlash()));
    }
    private double calculateFF(double encoder1Rad) {
      double ArmMoment = Config.ArmConfig.ARM_FORCE * (Config.ArmConfig.LENGTH_ARM_TO_COG*Math.cos(encoder1Rad));
      return (ArmMoment) * m_armMomentToVoltage.get();
    }

    public void isAtSetpoint() {
      
    }
  } 

