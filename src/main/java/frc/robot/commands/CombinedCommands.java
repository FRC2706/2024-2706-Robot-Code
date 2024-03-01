package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterStateMachine.ShooterModes;
import frc.robot.subsystems.ShooterSubsystem;

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
            new Shooter_Voltage(()->8)
        );
    }

    /**
     * Use state-based coding to shoot the note.
     */
    public static Command statefulShootNote() {
        return Commands.sequence( //Shoots the Note automatically 
            Commands.runOnce(()->{}),
            ShooterSubsystem.getInstance().speedUpForSpeakerCommand(),
            Commands.runOnce(()->{}),
            IntakeSubsystem.getInstance().shootNoteCommand(),
            Commands.runOnce(()->ShooterSubsystem.getInstance().setMode(ShooterModes.STOP_SHOOTER))          
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