package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;
import frc.lib.lib2706.ErrorCheck;
import frc.lib.lib2706.ProfiledPIDFFController;

public class ArmSubsystem extends SubsystemBase{
     private static ArmSubsystem instance = null; //static object that contains all movement controls

     private static final MotorType motorType = MotorType.kBrushless; //defines brushless motortype
     public final CANSparkMax m_Arm; //bottom SparkMax motor controller
     
     //network table entry
     private final String m_tuningTable = "Arm/ArmTuning";
     private final String m_dataTable = "Arm/ArmData";
  
     //network table entries 
     private DoubleEntry m_armPSubs;
     private DoubleEntry m_ArmISubs;
     private DoubleEntry m_ArmDSubs;
     private DoubleEntry m_ArmIzSubs;
     private DoubleEntry m_ArmFFSubs;
     private DoublePublisher m_ArmSetpointPub;   
     private DoublePublisher m_ArmVelPub;
     private DoubleEntry m_ArmMomentToVoltage;
     private DoublePublisher m_ArmFFTestingVolts;
     private DoubleEntry m_ArmOffset;
     private DoublePublisher m_TargetAngle;
     private DoublePublisher m_ArmPosPub;

    //for  arm ff
    private DoubleSubscriber momentToVoltageConversion;
    private double m_VoltageConversion;

    //spark absolute encoder
    SparkAbsoluteEncoder m_AbsEncoder;  
    //embedded relative encoder
    //private RelativeEncoder m_Encoder;
    public SparkPIDController m_pidControllerArm;  

    ProfiledPIDFFController m_profiledFFController = new ProfiledPIDFFController();

    public static ArmSubsystem getInstance() {
      if (instance == null) {
        SubsystemChecker.subsystemConstructed(SubsystemType.ArmSubsystem);
        instance = new ArmSubsystem();
      }
      return instance;
    }
    public ArmSubsystem() {
      m_Arm = new CANSparkMax(Config.CANID.ARM_SPARK_CAN_ID, motorType); //creates SparkMax motor controller 
      ErrorCheck.errREV(m_Arm.restoreFactoryDefaults());
      ErrorCheck.errREV(m_Arm.setSmartCurrentLimit(Config.ArmConfig.CURRENT_LIMIT));
      m_Arm.setInverted(Config.ArmConfig.SET_INVERTED); //sets movement direction
      ErrorCheck.errREV(m_Arm.setIdleMode(IdleMode.kBrake)); //sets brakes when there is no motion
      
      ErrorCheck.errREV(m_Arm.setSoftLimit(SoftLimitDirection.kForward, Config.ArmConfig.arm_forward_limit));
      ErrorCheck.errREV(m_Arm.setSoftLimit(SoftLimitDirection.kReverse, Config.ArmConfig.arm_reverse_limit));
      ErrorCheck.errREV(m_Arm.enableSoftLimit(SoftLimitDirection.kForward, Config.ArmConfig.SOFT_LIMIT_ENABLE));
      ErrorCheck.errREV(m_Arm.enableSoftLimit(SoftLimitDirection.kReverse, Config.ArmConfig.SOFT_LIMIT_ENABLE));

      m_Arm.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
      m_Arm.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

      m_AbsEncoder = m_Arm.getAbsoluteEncoder(Type.kDutyCycle);
      m_AbsEncoder.setInverted(true);
      m_AbsEncoder.setPositionConversionFactor(Config.ArmConfig.armPositionConversionFactor);
      m_AbsEncoder.setVelocityConversionFactor(Config.ArmConfig.armVelocityConversionFactor);
      m_AbsEncoder.setZeroOffset(Math.toRadians(Config.ArmConfig.armAbsEncoderOffset));

      m_pidControllerArm = m_Arm.getPIDController();
      m_pidControllerArm.setFeedbackDevice(m_AbsEncoder);

      
    NetworkTable ArmTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTable);
    m_ArmPSubs = ArmTuningTable.getDoubleTopic("P").getEntry(Config.ArmConfig.arm_kP, PubSubOption.periodic(0.02));
    m_ArmISubs = ArmTuningTable.getDoubleTopic("I").getEntry(Config.ArmConfig.arm_kI);
    m_ArmDSubs = ArmTuningTable.getDoubleTopic("D").getEntry(Config.ArmConfig.arm_kD);
    m_ArmIzSubs = ArmTuningTable.getDoubleTopic("IZone").getEntry(Config.ArmConfig.arm_kIz);
    m_ArmFFSubs = ArmTuningTable.getDoubleTopic("FF").getEntry(Config.ArmConfig.arm_kFF);
   //m_topArmOffset = topArmTuningTable.getDoubleTopic("Offset").getEntry(ArmConfig.top_arm_offset);
    momentToVoltageConversion = ArmTuningTable.getDoubleTopic("VoltageConversion").subscribe(m_VoltageConversion);

    m_ArmFFSubs.setDefault(Config.ArmConfig.arm_kFF);
    m_ArmPSubs.setDefault(Config.ArmConfig.arm_kP);
    m_ArmISubs.setDefault(Config.ArmConfig.arm_kI);
    m_ArmDSubs.setDefault(Config.ArmConfig.arm_kD);
    m_ArmIzSubs.setDefault(Config.ArmConfig.arm_kIz);

    NetworkTable ArmDataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);

    
    m_ArmPosPub = ArmDataTable.getDoubleTopic("MeasuredAngleDeg").publish(PubSubOption.periodic(0.02));
    m_TargetAngle = ArmDataTable.getDoubleTopic("TargetAngle").publish(PubSubOption.periodic(0.02));

    ErrorCheck.errREV(m_pidControllerArm.setP(m_ArmPSubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setI(m_ArmISubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setD(m_ArmDSubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setIZone(m_ArmIzSubs.get())); 
    ErrorCheck.errREV(m_pidControllerArm.setOutputRange(Config.ArmConfig.min_output, Config.ArmConfig.max_output));

    updatePIDSettings();
    //updateFromAbsoluteBottom();
  }
  public void updatePIDSettings() {
    ErrorCheck.errREV(m_pidControllerArm.setFF(m_ArmFFSubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setP(m_ArmPSubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setI(m_ArmISubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setD(m_ArmDSubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setIZone(m_ArmIzSubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setOutputRange(Config.ArmConfig.min_output, Config.ArmConfig.max_output));
  }
  @Override
  public void periodic() {
    m_ArmPosPub.accept(Math.toDegrees(m_AbsEncoder.getPosition()));
    m_ArmVelPub.accept(m_AbsEncoder.getVelocity());
  }
    //input angle_bottom in radians
    public void setJointAngle(double angle) {
      if (angle<Math.toRadians(Config.ArmConfig.MIN_ARM_ANGLE_DEG) || angle>Math.toRadians(Config.ArmConfig.MAX_ARM_ANGLE_DEG)) {
        angle = Math.toRadians(Config.ArmConfig.MAX_ARM_ANGLE_DEG);
      }
    
      double targetPos = m_profiledFFController.getNextProfiledPIDPos(getPosition(), angle);
      m_pidControllerArm.setReference((targetPos), ControlType.kPosition, 0, calculateFF(angle));
      m_TargetAngle.accept(Math.toDegrees(targetPos));
      System.out.println(targetPos);
    }

  
    //return radius
    public double getPosition() {
      return m_AbsEncoder.getPosition();
    }
  
    public void stopMotors() {
      m_Arm.stopMotor();
    }
    public void burnFlash() {
      ErrorCheck.errREV(m_Arm.burnFlash());
    }
    private double calculateFF(double encoder1Rad) {
      double ArmMoment = Config.ArmConfig.ARM_FORCE * (Config.ArmConfig.LENGTH_ARM_TO_COG*Math.cos(encoder1Rad));
      return (ArmMoment) * m_ArmMomentToVoltage.get();
    }
  } 

