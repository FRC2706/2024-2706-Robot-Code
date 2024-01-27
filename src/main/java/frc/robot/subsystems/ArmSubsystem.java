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
import frc.robot.ErrorCheck;
import frc.robot.ProfiledPIDFFController;

public class ArmSubsystem extends SubsystemBase{
     private static ArmSubsystem instance = null; //static object that contains all movement controls

     private static final MotorType motorType = MotorType.kBrushless; //defines brushless motortype
     public final CANSparkMax m_Arm; //bottom SparkMax motor controller
     
     //network table entry
     private final String m_tuningTable = "Arm/ArmTuning";
     private final String m_dataTable = "Arm/BottomArmData";
  
     //network table entries 
     private DoubleEntry m_ArmPSubs;
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
    //private RelativeEncoder m_bottomEncoder;
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
      m_Arm = new CANSparkMax(Config.CANID.ARM_SPARK_CAN_ID, motorType); //creates SparkMax motor controller for bottom joint
      ErrorCheck.errREV(m_Arm.restoreFactoryDefaults());
      ErrorCheck.errREV(m_Arm.setSmartCurrentLimit(ArmConfig.CURRENT_LIMIT));
      m_Arm.setInverted(ArmConfig.SET_INVERTED); //sets movement direction
      ErrorCheck.errREV(m_Arm.setIdleMode(IdleMode.kBrake)); //sets brakes when there is no motion
      
      ErrorCheck.errREV(m_Arm.setSoftLimit(SoftLimitDirection.kForward, ArmConfig.arm_forward_limit));
      ErrorCheck.errREV(m_Arm.setSoftLimit(SoftLimitDirection.kReverse, ArmConfig.arm_reverse_limit));
      ErrorCheck.errREV(m_Arm.enableSoftLimit(SoftLimitDirection.kForward, ArmConfig.SOFT_LIMIT_ENABLE));
      ErrorCheck.errREV(m_Arm.enableSoftLimit(SoftLimitDirection.kReverse, ArmConfig.SOFT_LIMIT_ENABLE));

      m_Arm.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
      m_Arm.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

      m_AbsEncoder = m_Arm.getAbsoluteEncoder(Type.kDutyCycle);
      m_AbsEncoder.setInverted(true);
      m_AbsEncoder.setPositionConversionFactor(2*Math.PI);
      m_AbsEncoder.setVelocityConversionFactor(2*Math.PI/60.0);
      m_AbsEncoder.setZeroOffset(Math.toRadians(52));

      m_pidControllerArm = m_Arm.getPIDController();
      m_pidControllerArm.setFeedbackDevice(m_AbsEncoder);

      
    NetworkTable ArmTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTable);
    m_ArmPSubs = ArmTuningTable.getDoubleTopic("P").getEntry(ArmConfig.arm_kP, PubSubOption.periodic(0.02));
    m_ArmISubs = ArmTuningTable.getDoubleTopic("I").getEntry(ArmConfig.arm_kI);
    m_ArmDSubs = ArmTuningTable.getDoubleTopic("D").getEntry(ArmConfig.arm_kD);
    m_ArmIzSubs = ArmTuningTable.getDoubleTopic("IZone").getEntry(ArmConfig.arm_kIz);
    m_ArmFFSubs = ArmTuningTable.getDoubleTopic("FF").getEntry(ArmConfig.arm_kFF);
   //m_topArmOffset = topArmTuningTable.getDoubleTopic("Offset").getEntry(ArmConfig.top_arm_offset);
    momentToVoltageConversion = ArmTuningTable.getDoubleTopic("VoltageConversion").subscribe(m_VoltageConversion);

    m_ArmFFSubs.accept(ArmConfig.arm_kFF);
    m_ArmPSubs.accept(ArmConfig.arm_kP);
    m_ArmISubs.accept(ArmConfig.arm_kI);
    m_ArmDSubs.accept(ArmConfig.arm_kD);
    m_ArmIzSubs.accept(ArmConfig.arm_kIz);

    NetworkTable ArmDataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);

    
    m_ArmPosPub = ArmDataTable.getDoubleTopic("MeasuredAngle").publish(PubSubOption.periodic(0.02));
    m_TargetAngle = ArmDataTable.getDoubleTopic("TargetAngle").publish(PubSubOption.periodic(0.02));
    m_ArmVelPub = ArmDataTable.getDoubleTopic("Vel").publish(PubSubOption.periodic(0.02));

    
    ErrorCheck.errREV(m_pidControllerArm.setFF(m_ArmFFSubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setP(m_ArmPSubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setI(m_ArmISubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setD(m_ArmDSubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setIZone(m_ArmIzSubs.get())); 
    ErrorCheck.errREV(m_pidControllerArm.setOutputRange(ArmConfig.min_output, ArmConfig.max_output));

    updatePIDSettings();
    //updateFromAbsoluteBottom();
  }
  public void updatePIDSettings() {
    ErrorCheck.errREV(m_pidControllerArm.setFF(m_ArmFFSubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setP(m_ArmPSubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setI(m_ArmISubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setD(m_ArmDSubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setIZone(m_ArmIzSubs.get()));
    ErrorCheck.errREV(m_pidControllerArm.setOutputRange(ArmConfig.min_output, ArmConfig.max_output));
  }
  @Override
  public void periodic() {
    m_ArmPosPub.accept(Math.toDegrees(m_AbsEncoder.getPosition()));
    m_ArmVelPub.accept(m_AbsEncoder.getVelocity());
    m_TargetAngle.accept(Math.toDegrees(getAbsolute()));
  }
    //input angle_bottom in radians
    public void setBottomJointAngle(double angle_bottom) {
      if (angle_bottom<Math.toRadians(0) || angle_bottom>Math.toRadians(95)) {
        angle_bottom = Math.toRadians(95);
      }
      //setReference angle is in radians)
      //todo: tune FF 
      m_pidControllerArm.setReference((angle_bottom), ControlType.kPosition, 0,0.1);
    }
  
    public void setTopJointAngle(double angle_top) {
      if (angle_top<Math.toRadians(0) || angle_top>Math.toRadians(35)) {
        angle_top = Math.toRadians(10);
      }
      //setReference angle is in radians)
      //todo: tune FF 
      double targetPos = m_profiledFFController.getNextProfiledPIDPos(getPosition(), angle_top);
      m_pidControllerArm.setReference((targetPos), ControlType.kPosition, 0, calculateFFTop());
      m_TargetAngle.accept(Math.toDegrees(targetPos));
      System.out.println(targetPos);
  
    }
  
    /*public void controlBottomArmBrake( boolean bBrakeOn) {
      if (bBrakeOn == true) {
        //set brake on the arm 
        brakeSolenoidLow.set(Value.kForward);
      }
      else {
        brakeSolenoidLow.set(Value.kReverse);
      }
    }
    */
  
    //return radius
    public double getPosition() {
      return m_AbsEncoder.getPosition();
    }
  
    public double getAbsolute() {
      //getAbsolutePosition() return in [0,1]
     //return Math.toRadians(m_bottomDutyCycleEncoder.getAbsolutePosition() * -360 + m_bottomArmOffset.get());
    //double absPosition = m_bottomAbsEncoder.getPosition();
    //double adjAbsPosition = absPosition-((int) absPosition/360)*360.0;
     //System.out.println("Get abs position(degree) " + absPosition);
     //System.out.println("adjusted abs position " + adjAbsPosition);
     return m_AbsEncoder.getPosition(); //+ m_bottomArmOffset.get());
    }
  
    public boolean areEncodersSynced() {
      System.out.println("*****areEncodersSynced*****");
      boolean syncResult;
        //set sparkmax encoder position
      //updateFromAbsoluteBottom();
      System.out.println("getAbsoluteBottom " + getAbsolute());
      System.out.println("getBottomPosition " + getPosition());
      syncResult = Math.abs(getAbsolute() - getPosition()) < ArmConfig.ENCODER_SYNCING_TOLERANCE;
      System.out.println("******SyncIteration****** " + "******Result******"  + syncResult);
      return syncResult; 
    }
    public void stopMotors() {
      m_Arm.stopMotor();
      m_Arm.stopMotor();
    }
    public void burnFlash() {
      ErrorCheck.errREV(m_Arm.burnFlash());
      ErrorCheck.errREV(m_Arm.burnFlash());
    }
    private double calculateFFTop() {
      double enc2AtHorizontal = getPosition() - (Math.PI - getPosition());
      double voltsAtHorizontal;
      voltsAtHorizontal = 2.0;
      //System.out.println("top position " + getTopPosition());
      //System.out.println("bottom position " + getBottomPosition());
      //System.out.println("calculated position " + enc2AtHorizontal);
      System.out.println("return voltage " + voltsAtHorizontal * Math.cos(enc2AtHorizontal));
      return voltsAtHorizontal * Math.cos(enc2AtHorizontal);
    } 
}
