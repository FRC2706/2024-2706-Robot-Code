package frc.robot.commands;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.lib2706.SelectByAllianceCommand;
import frc.robot.Config.Swerve;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterStateMachine.ShooterModes;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

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
     * Runs the given command. If the given command ends before the timeout, the command will end as normal.
     * If the timeout happens before the command ends, this command will forceful cancel itself and any 
     * future command groups it's apart of.
     * 
     * This means if the next command in a sequence is to shoot, it won't shoot unless the given command here
     * has correctly ended before the timeout.
     * 
     * @param timeout in seconds
     * @param command to run
     * @return
     */
    public static Command forcefulTimeoutCommand(double timeoutSeconds, Command command) {
        // Create a command that has the same requirements as the given command.
        Command requirementCommand = new InstantCommand();
        for (Subsystem s : command.getRequirements()) {
            requirementCommand.addRequirements(s);
        }


        // If WaitCommand ends before the given command ends, schedule a command to forceful cancel this 
        // command group and any future commands this command group is apart of (like a command to shoot)
        // Else if the given command ends before the timeout, continue on as normal.
        return Commands.race(
            new WaitCommand(timeoutSeconds).andThen(new ScheduleCommand(requirementCommand)),
            command
        );
    }

    /**
     * Score in the amp or speaker using vision and the given parameters.
     * Uses simple programming for the intake and shooter.
     * Handles arm, intake, shooter, swerve, and vision.
     * 
     * @param driverJoystick A CommandXboxController
     * @param preparingTimeoutSeconds Safety timeout for the robot to move to the correct position, ready the arm and ready the intake
     * @param scoringTimeoutSeconds Run shooter and intake until this timeout is reached
     * @param armAngle Arm angle in degrees to use
     * @param shooterVoltage Voltage of the shooter
     * @param bluePosition PhotonPosition for the blue alliance
     * @param redPosition PhotonPosition for the red alliance
     */ 
    public static Command visionScoreTeleopSimple(
            CommandXboxController driverJoystick, 
            double preparingTimeoutSeconds, 
            double scoringTimeoutSeconds, 
            double armAngleDeg, 
            double shooterVoltage, 
            PhotonPositions bluePosition, 
            PhotonPositions redPosition) {

        // Swerve requirement command
        Command idleSwerve = Commands.idle(SwerveSubsystem.getInstance()).withName("IdlingSwerveSimple");

        // Prepare the robot to score
        Command driveToPositionAndPrepare = Commands.deadline(
            Commands.parallel(
                new SetArm(()->armAngleDeg),
                new IntakeControl(false), // Reverse note until not touching shooter
                new WaitCommand(0.7), // Require a minimum duration for shooter to spinup
                Commands.sequence(
                    new SelectByAllianceCommand(
                        PhotonSubsystem.getInstance().getAprilTagCommand(bluePosition, driverJoystick), 
                        PhotonSubsystem.getInstance().getAprilTagCommand(redPosition, driverJoystick)),
                    new ScheduleCommand(idleSwerve) // Hog the swerve subsystem to prevent the teleop command from running
                ) 
            ),
            new Shooter_Voltage(() -> shooterVoltage) // Shooter ends when the the commands above
        );

        // Score the note
        Command scoreNote = Commands.parallel(
            Commands.runOnce(() -> SwerveSubsystem.getInstance().stopMotors()),
            new Shooter_Voltage(() -> shooterVoltage),
            new IntakeControl(true),
            new SetArm(()->armAngleDeg) // Continue to hold arm in the correct position
        ).withTimeout(scoringTimeoutSeconds);

        // Rumble command
        Command rumble = new RumbleJoystick(driverJoystick, RumbleType.kBothRumble, 0.7, 0.2, false);

        // Sequence preparing then scoring
        return Commands.sequence( 
            forcefulTimeoutCommand(
                preparingTimeoutSeconds,
                driveToPositionAndPrepare
            ),
            scoreNote
        ).finallyDo(() -> {
            rumble.schedule(); // Rumble the joystick to notify the driver
            idleSwerve.cancel(); // Ensure the teleop command is not blocked
        });
    }

    /**
     * Score in the amp or speaker using vision and the given parameters.
     * Uses state based programming for the intake and shooter.
     * Handles arm, intake, shooter, swerve, and vision. 
     *
     * @param driverJoystick A CommandXboxController
     * @param preparingTimeoutSeconds Safety timeout for the robot to move to the correct position, ready the arm and ready the intake
     * @param scoringTimeoutSeconds Safety timeout for scoring after the robot is prepared
     * @param armAngle Arm angle in degrees to use
     * @param bluePosition PhotonPosition for the blue alliance
     * @param redPosition PhotonPosition for the red alliance
     */
    public static Command visionScoreTeleopStateful(
            CommandXboxController driverJoystick, 
            double preparingTimeoutSeconds,
            double scoringTimeoutSeconds, 
            double armAngleDeg, 
            PhotonPositions bluePosition, 
            PhotonPositions redPosition) {
         
        // Swerve requirement command
        Command idleSwerve = Commands.idle(SwerveSubsystem.getInstance()).withName("IdlingSwerveStateful");

        // Prepare the robot to score
        Command driveToPositionAndPrepare = Commands.parallel(
            new SetArm(()->armAngleDeg),
            new WaitCommand(0.1).andThen(ShooterSubsystem.getInstance().speedUpForSpeakerCommand()),
            Commands.sequence(
                new SelectByAllianceCommand(
                    PhotonSubsystem.getInstance().getAprilTagCommand(bluePosition, driverJoystick), 
                    PhotonSubsystem.getInstance().getAprilTagCommand(redPosition, driverJoystick)),
                new ScheduleCommand(idleSwerve) // Hog the swerve subsystem to prevent the teleop command from running
            ) 
        ); 

        // Score the note
        Command scoreNote = Commands.parallel(
            Commands.runOnce(() -> SwerveSubsystem.getInstance().stopMotors()),
            IntakeSubsystem.getInstance().shootNoteCommand(),
            new SetArm(()->armAngleDeg) // Continue to hold arm in the correct position
        ).withTimeout(scoringTimeoutSeconds);

        // Rumble command
        Command rumble = new RumbleJoystick(driverJoystick, RumbleType.kBothRumble, 0.7, 0.2, false);

        // Sequence preparing then scoring
        return Commands.sequence( 
            forcefulTimeoutCommand(
                preparingTimeoutSeconds,
                driveToPositionAndPrepare
            ),
            scoreNote,
            Commands.runOnce(() -> ShooterSubsystem.getInstance().setMode(ShooterModes.STOP_SHOOTER)) 
        ).finallyDo(() -> {
            rumble.schedule(); // Rumble the joystick to notify the driver
            idleSwerve.cancel(); // Ensure the teleop command is not blocked
        });
    }
}