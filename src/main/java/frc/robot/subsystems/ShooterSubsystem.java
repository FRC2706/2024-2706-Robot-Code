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
import frc.lib.lib2706.ErrorCheck;
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


    private TunableNumber kP = new TunableNumber("Shooter/PID0/kP", Config.ShooterConstants.kP);
    private TunableNumber kI = new TunableNumber("Shooter/PID0/kI", Config.ShooterConstants.kI);
    private TunableNumber kD = new TunableNumber("Shooter/PID0/kD", Config.ShooterConstants.kD);
    private TunableNumber kFF = new TunableNumber("Shooter/PID0/kFF", Config.ShooterConstants.kFF);

    private TunableNumber kP1 = new TunableNumber("Shooter/PID1/kP", Config.ShooterConstants.kP1);
    private TunableNumber kI1 = new TunableNumber("Shooter/PID1/kI", Config.ShooterConstants.kI1);
    private TunableNumber kD1 = new TunableNumber("Shooter/PID1/kD", Config.ShooterConstants.kD1);
    private TunableNumber kFF1 = new TunableNumber("Shooter/PID1/kFF", Config.ShooterConstants.kFF1);
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
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(false);

        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();

        //Voltage compensation
        m_motor.enableVoltageCompensation(10); //adjust on final robot
        m_motor.setSmartCurrentLimit(70);  
        setBrake(true);

        m_pidController.setOutputRange(Config.ShooterConstants.kMinOutput, Config.ShooterConstants.kMaxOutput);
        setPIDGains(kP.get(), kI.get(), kD.get(), 0);
        setFFGains(kFF.get(), 0);

        setPIDGains(kP1.get(), kI1.get(), kD1.get(), 1);
        setFFGains(kFF1.get(), 1);

        ErrorCheck.sparkBurnFlash("Shooter", m_motor);

        NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("Shooter");
        velocityPub = shooterTable.getDoubleTopic("Shooter Velocity RPM").publish(PubSubOption.periodic(0.02));
        shooterReadyPub = shooterTable.getBooleanTopic("Shooter is Ready to shoot").publish(PubSubOption.periodic(0.02));
        statePub = shooterTable.getStringTopic("Shooter state").publish(PubSubOption.periodic(0.02));

        ErrorTrackingSubsystem.getInstance().register(m_motor);
    }

    public double getVelocityRPM() {
        return m_encoder.getVelocity();
    }

    public void setRPM(double setPoint) {
        int slotID = 1;
        m_pidController.setReference(setPoint, ControlType.kVelocity, slotID);
    }

    public void setVoltage(double setVolt) {
        m_motor.setVoltage(setVolt);
    }

    public void stop(){
        m_motor.stopMotor();
    }

    public void setMode(ShooterModes desiredMode){
        shooterStates.setMode(desiredMode);
    }

    public void allowAutoMovement(boolean isThereNote){
        if(!isThereNote && stateFulControl)setMode(STOP_SHOOTER);//it should set to stop now when there is no note in intake

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

    private void setPIDGains(double kP, double kI, double kD, int slotID){
        m_pidController.setP(kP, slotID);
        m_pidController.setI(kI, slotID);
        m_pidController.setD(kD, slotID);
    }   

    private void setFFGains(double kFF, int slotID){
        m_pidController.setFF(kFF, slotID);
    }   

    public boolean isReadyToShoot(){
        return getCurrentState().equals(SPEAKER_LAUNCH_READY) || getCurrentState().equals(AMP_LAUNCH_READY);
    }

    public void setStateMachineOff(){
        stateFulControl = false;
    }

    public void setStateMachineOn(){
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

    /**
     * Command that will set the the given mode if shooter is stopped,
     * or stop the shooter if it's currently doing an action.
     * 
     * @param mode Mode to toggle.
     * @return Command to attach to a button as onTrue.
     */
    public Command toggleSpinUpCommand(ShooterModes mode) {
        return Commands.runOnce(() -> {
            if (shooterStates.getDesiredMode() != ShooterModes.STOP_SHOOTER) {
                shooterStates.setMode(ShooterModes.STOP_SHOOTER);
            } else {
                shooterStates.setMode(mode);
            }
        });
    }
    
    @Override
    public void periodic() {
        TunableNumber.ifChanged(hashCode(), ()->setPIDGains(kP.get(), kI.get(), kD.get(), 0), kP, kI, kD);
        TunableNumber.ifChanged(hashCode(), ()->setFFGains(kFF.get(), 0), kFF);

        TunableNumber.ifChanged(hashCode(), ()->setPIDGains(kP1.get(), kI1.get(), kD1.get(), 1), kP1, kI1, kD1);
        TunableNumber.ifChanged(hashCode(), ()->setFFGains(kFF1.get(), 1), kFF1);

        //Check if this method would work like this
        if(stateFulControl == true) {
            shooterStates.isInRange(()->getVelocityRPM() > shooterStates.getDesiredVelocityRPM() - shooterTreshHold.get());
            shooterStates.updateState();
        }
        

        velocityPub.accept(getVelocityRPM());
        shooterReadyPub.accept(isReadyToShoot());
        statePub.accept(getCurrentState().toString());
    }
}
