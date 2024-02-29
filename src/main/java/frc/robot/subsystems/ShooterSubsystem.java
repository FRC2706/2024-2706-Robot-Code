package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.subsystems.ShooterStateMachine.ShooterModes.*;
import static frc.robot.subsystems.ShooterStateMachine.States.*;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.TunableNumber;
import frc.robot.Config;
import frc.robot.subsystems.ShooterStateMachine.ShooterModes;
import frc.robot.subsystems.ShooterStateMachine.States;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax m_motor;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private boolean closedLoopControl = false;
    private boolean stateFulControl = false; 

    private TunableNumber kP = new TunableNumber("Shooter/kP", Config.ShooterConstants.kP);
    private TunableNumber kI = new TunableNumber("Shooter/kI", Config.ShooterConstants.kI);
    private TunableNumber kD = new TunableNumber("Shooter/kD", Config.ShooterConstants.kD);
    private TunableNumber kFF = new TunableNumber("Shooter/kFF", Config.ShooterConstants.kFF);
    private TunableNumber shooterTreshHold = new TunableNumber("Shooter/tresh hold", 100);
    
    private DoublePublisher velocityPub;
    private StringPublisher statePub;
    private BooleanPublisher shooterReadyPub;
    private ShooterStateMachine shooterStates = new ShooterStateMachine();

    private static ShooterSubsystem shooter;
    public static ShooterSubsystem getInstance() {
        if (shooter == null)
            shooter = new ShooterSubsystem();
        return shooter;
    }

    public ShooterSubsystem() {
        System.out.println("[Init] Creating Shooter");
        m_motor = new CANSparkMax(Config.ShooterConstants.MOTOR_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();

        m_motor.setCANTimeout(500);//Units in miliseconds
        m_motor.setIdleMode(IdleMode.kCoast);
        m_motor.setInverted(false);

        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();   
        m_encoder.setAverageDepth(1);//check if 2 would work better

        //Voltage compensation
        m_motor.enableVoltageCompensation(12); //adjust on final robot
        m_motor.setSmartCurrentLimit(40);  
        setBrake(false);

        m_pidController.setOutputRange(Config.ShooterConstants.kMinOutput, Config.ShooterConstants.kMaxOutput);
        setPIDGains(kP.get(), kI.get(), kD.get());
        setFFGains(kFF.get());

        NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("Shooter");
        velocityPub = shooterTable.getDoubleTopic("Shooter Velocity RPM").publish(PubSubOption.periodic(0.02));
        shooterReadyPub = shooterTable.getBooleanTopic("Shooter is Ready to shoot").publish(PubSubOption.periodic(0.02));
        statePub = shooterTable.getStringTopic("Shooter state").publish(PubSubOption.periodic(0.02));
    }

    public double getVelocityRPM() {
        return m_encoder.getVelocity();
    }

    public void setRPM(double setPoint) {
        m_pidController.setReference(setPoint, ControlType.kVelocity);
    }

    public void setVoltage(double setVolt) {
        m_motor.setVoltage(setVolt);
    }

    public void stop(){
        setVoltage(0);
    }

    public void setMode(ShooterModes desiredMode){
        shooterStates.setMode(desiredMode);
    }

    public void allowAutoMovement(boolean isThereNote){
        if(!isThereNote)setMode(STOP_SHOOTER);

        if(closedLoopControl){
            setRPM(shooterStates.getDesiredVelocityRPM());
        }else{
            setVoltage(shooterStates.getDesiredVoltage());
        }
    }

    public void setBrake(boolean enableBreak){
        m_motor.setIdleMode(enableBreak ? IdleMode.kBrake: IdleMode.kCoast);
    }

    public States getCurrentState(){
        return shooterStates.getCurrentState();
    }

    private void setPIDGains(double kP, double kI, double kD){
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
    }   

    private void setFFGains(double kFF){
        m_pidController.setFF(kFF);
    }   

    public boolean isReadyToShoot(){
        return getCurrentState().equals(SPEAKER_LAUNCH_READY) || getCurrentState().equals(AMP_LAUNCH_READY);
    }

    public void changeToNonStateFull(){
        stateFulControl = false;
    }

    public void changeToStateFull(){
        stateFulControl = true;
    }

    /*---------------------------Commands---------------------------*/

    /**
     * This allows the Shooter's state machine to have effect on the beheavior of the shooter
     * This should be called every run loop cycle, set it as the default command 
     * @return Default Intake Command
     */
    public Command defaultShooterCommand(BooleanSupplier isThereNote){
        return Commands.sequence(
            runOnce(()->setMode(STOP_SHOOTER)), run(()->allowAutoMovement(isThereNote.getAsBoolean())));
    }
    /**
     * Sets the mode of the Shooter's state nachine to "SHOOT_SPEAKER"
     * This will set the velocity to the 
     * @return
     */
    public Command speedUpForSpeakerCommand(){
        return Commands.deadline(
                Commands.waitUntil(()->isReadyToShoot()), 
                Commands.runOnce(()->setMode(SHOOT_SPEAKER))
            );
    }
    
    @Override
    public void periodic() {
        TunableNumber.ifChanged(hashCode(), ()->setPIDGains(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        TunableNumber.ifChanged(hashCode(), ()->setFFGains(kFF.get()), kFF);

        //Check if this method would work like this
        shooterStates.isInRange(()->getVelocityRPM() > shooterStates.getDesiredVelocityRPM() - shooterTreshHold.get());
        if(stateFulControl){
            shooterStates.updateState();
        }

        velocityPub.accept(getVelocityRPM());
        shooterReadyPub.accept(isReadyToShoot());
        statePub.accept(getCurrentState().toString());
    }
}
