package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class MakeIntakeMotorSpin extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    private double targetSpeed;

    public MakeIntakeMotorSpin(Double Speed) {
        targetSpeed = Speed;


        intakeSubsystem = IntakeSubsystem.getInstance();

        if (intakeSubsystem != null) {
            addRequirements(intakeSubsystem);
        }

    @Override
    public void execute() {
        if ( intakeSubsystem != null ) {
            intakeSubsystem.setMotorRPM(targetSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if ( intakeSubsystem != null ) {
            intakeSubsystem.stop();
        }
    }



    }

}
