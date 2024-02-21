package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class MakeIntakeMotorSpin extends Command {

    private IntakeSubsystem intakeSubsystem;
    private double targetVoltage;
    private int timeout;
    private Timer timer;
    private boolean m_bUseTimer;


    public MakeIntakeMotorSpin(Double Voltage, int i) {
        targetVoltage = Voltage;
        timeout = i;

        if (timeout > 0) {
            m_bUseTimer = true;
            timer = new Timer();
        } else {
            m_bUseTimer = false;
        }


        intakeSubsystem = IntakeSubsystem.getInstance();

        if (intakeSubsystem != null) {
            addRequirements(intakeSubsystem);
        }

    } 


    @Override
    public void initialize() {
        if(m_bUseTimer == true) {
            timer.start();
            timer.reset();
        }
    }

    @Override
    public void execute() {
        if ( intakeSubsystem != null ) {
            intakeSubsystem.setVoltage(targetVoltage);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if ( intakeSubsystem != null ) {
            intakeSubsystem.stop();
        }

        if (m_bUseTimer == true) {
            timer.stop();
        }

    } 


    @Override
    public boolean isFinished() {
        if (m_bUseTimer == true) {
            return timer.get() > timeout;
        } else {
            return false;
        }
    }


}

