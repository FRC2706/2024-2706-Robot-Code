package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.Shooter_tuner;
import frc.robot.subsystems.IntakeStatesVoltage;
import frc.robot.subsystems.IntakeSubsystem;

public class CombinedCommands {
    /**
     * This file should not be constructed. It should only have static factory methods.
     */
    private CombinedCommands () {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Spin up the shooter while doing the following,
     * backing up note, waiting a bit, then feeding the note.
     */
    public static Command simpleShootNote() {
        return Commands.deadline(
            Commands.sequence(
                new IntakeControl(false).withTimeout(0.3), 
                new WaitCommand(0.5),
                new IntakeControl(true).withTimeout(2)),
            new Shooter_tuner(()->5)
        );
    }

    /**
     * Use state-based coding to shoot the note.
     */
    public static Command statefulShootNote() {
        return Commands.sequence(
            Commands.runOnce(() -> {System.out.println("STEP1");}),
            // Shooter.getInstance().prepare4Speaker(),
            Commands.runOnce(() -> {System.out.println("STEP2");}),
            IntakeSubsystem.getInstance().shootNote(),
            Commands.runOnce(() -> {System.out.println("STEP3");})//,
            // Commands.runOnce(()->Shooter.getInstance().setMode(ShooterStateVoltage.Modes.STOP_SHOOTER))          
        );
    }   

    /**
     * Score in the amp using vision.
     * Handles arm, intake, shooter, swerve and vision.
     */
    public static Command visionScoreAmpTeleopSimple() {
        return Commands.sequence(
            
        );
    }

    

}
