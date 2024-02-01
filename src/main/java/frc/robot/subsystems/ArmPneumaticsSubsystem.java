package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class ArmPneumaticsSubsystem  extends SubsystemBase{
    private DoubleSolenoid brakeSolenoid;

    public Timer brakeTimer = new Timer();

    private static ArmPneumaticsSubsystem instance;
    public static ArmPneumaticsSubsystem getInstance(){
        if(instance == null){
            instance = new ArmPneumaticsSubsystem();
        }
        return instance;
    }

    // Create ArmPneumaticsSubsystem
    private ArmPneumaticsSubsystem(){
    brakeSolenoid = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Config.ARMLOW_PNEUMATIC_FORWARD_CHANNEL, Config.ARMLOW_PNEUMATIC_REVERSE_CHANNEL);
    }
    // control bottom brake
    public void controlBottomBrake(boolean brakeOn, boolean turnOffAfterHalfSecond) {
        if (brakeOn) {
            brakeSolenoid.set(Value.kForward);
        } else {
            brakeSolenoid.set(Value.kReverse);
        }
        if (turnOffAfterHalfSecond) {
            brakeTimer.restart();
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // check if bottom brake timer has elapsed
        if (brakeTimer.hasElapsed(0.5)) {
            brakeSolenoid.set(Value.kOff);
            brakeTimer.stop();
            brakeTimer.reset();
        }
    }
}

