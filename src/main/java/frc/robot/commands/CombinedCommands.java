package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.lib2706.SelectByAllianceCommand;
import frc.robot.Config.ArmSetPoints;
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
    public static Command simpleShootNoteSpeaker(double intakeTimeout) {
        return(simpleShootNoteSpeaker(intakeTimeout, () -> 3000, 400));
    }

    /**
     * Spin up the shooter while doing the following,
     * backing up note, waiting a bit, then feeding the note.
     */
    public static Command simpleShootNoteSpeaker(double intakeTimeout, DoubleSupplier RPM, double threshold) {
        return Commands.deadline(
            Commands.sequence(
                new IntakeControl(false).withTimeout(0.15), 
                new WaitUntilCommand(() -> ShooterSubsystem.getInstance().getVelocityRPM() > RPM.getAsDouble()),
                new IntakeControl(true).withTimeout(intakeTimeout)),
            new Shooter_PID_Tuner(()->(RPM.getAsDouble() + threshold))
        );
    }

    public static Command simpleShootNoteAmp() {
        return Commands.deadline(
            Commands.sequence(
                new IntakeControl(false).withTimeout(0.3), 
                new WaitCommand(0.5),
                new IntakeControl(true).withTimeout(0.6)),
            new Shooter_Voltage(()->6)
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

     // Intake and Arm Intake Position
    public static Command armIntake() {
        return Commands.parallel(
        new MakeIntakeMotorSpin(9.0,0),
        new SetArm(()->ArmSetPoints.INTAKE.angleDeg) // Continue to hold arm in the correct position
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
            double shooterSpeed,
            double shooterTriggerSpeed,
            double armAngleDeg, 
            PhotonPositions bluePosition, 
            PhotonPositions redPosition) {

        // Swerve requirement command
        Command idleSwerve = new ProxyCommand(Commands.idle(SwerveSubsystem.getInstance())).withName("IdlingSwerveSimple");

        // Wait for vision data to be available
        Command waitForVisionData = new ProxyCommand(new SelectByAllianceCommand(
            PhotonSubsystem.getInstance().getWaitForDataCommand(bluePosition.id), 
            PhotonSubsystem.getInstance().getWaitForDataCommand(redPosition.id)));
            
        // Prepare the robot to score
        Command driveToPositionAndPrepare = Commands.deadline(
            Commands.parallel(
                new IntakeControl(false), // Reverse note until not touching shooter
                new WaitUntilCommand(() -> ShooterSubsystem.getInstance().getVelocityRPM() > shooterTriggerSpeed),
                Commands.sequence(
                    new SelectByAllianceCommand(
                        PhotonSubsystem.getInstance().getAprilTagCommand(bluePosition, driverJoystick), 
                        PhotonSubsystem.getInstance().getAprilTagCommand(redPosition, driverJoystick))
                    // new ScheduleCommand(idleSwerve) // Maintain control of the SwerveSubsystem
                ) 
            ),
            new SetArm(()->armAngleDeg),
            new Shooter_PID_Tuner(() -> shooterSpeed)
        );

        // Score the note
        Command scoreNote = Commands.parallel(
            Commands.runOnce(() -> SwerveSubsystem.getInstance().stopMotors()),
            new Shooter_PID_Tuner(() -> shooterSpeed+300), // Continue to hold shooter voltage
            new SetArm(()->armAngleDeg), // Continue to hold arm in the correct position
            new MakeIntakeMotorSpin(9.0, 0)
        ).withTimeout(scoringTimeoutSeconds);

        // Rumble command
        Command rumble = new RumbleJoystick(driverJoystick, RumbleType.kBothRumble, 0.7, 0.2, false);

        // Sequence preparing then scoring
        return Commands.sequence( 
            waitForVisionData,
            forcefulTimeoutCommand(
                preparingTimeoutSeconds,
                driveToPositionAndPrepare
            ),
            scoreNote
        ).finallyDo(() -> {
            rumble.schedule(); // Rumble the joystick to notify the driver
            idleSwerve.cancel(); // Release control of swerve
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



    /**
     * Score in amp with vision using simple intake/shooter
     * @param driver joystick
     */
    public static Command simpleAmpScoreWithVision(CommandXboxController driver) {
        return CombinedCommands.visionScoreTeleopSimple(
            driver, 
            25, 
            2, 
            2000, 2500,
            ArmSetPoints.AMP.angleDeg,
            PhotonPositions.AMP_BLUE,
            PhotonPositions.AMP_RED
        );
    }    

    /**
     * Score in speaker with vision using simple intake/shooter.
     * 
     * @param driver joystick
     * @param bluePosition PhotonPosition for the blue alliance
     * @param redPosition PhotonPosition for the red alliance
     */
    public static Command sideSpeakerVisionShot(CommandXboxController driver, PhotonPositions bluePosition, PhotonPositions redPosition) {
        // BELOW VALUES WORKING AT KINGSTON BUT SHOOTER DEEMED TOO LOUD. CHANGE BACK TO THIS ONCE SOLVED MECHANICALLY.
        // double armAngle = 29;
        // double shooterSpeed = 4200;
        // double shooterTriggerSpeed = 3500;


        // BELOW VALUES SHOULD BE SAFE FOR NOISE, BUT LIKELY WILL MISS OR BE LESS CONSISTENT
        double armAngle = 26;
        double shooterSpeed = 3400;
        double shooterTriggerSpeed = 3000;

        return CombinedCommands.visionScoreTeleopSimple(
            driver, 
            25, 
            1, 
            shooterSpeed, shooterTriggerSpeed,
            armAngle,
            bluePosition,
            redPosition
        );
    }

    /**
     * Score in amp with vision using stateful intake/shooter
     * 
     * @param driver joystick
     */
    public static Command statefulAmpScoreWithVision(CommandXboxController driver) {
        return CombinedCommands.visionScoreTeleopStateful(
            driver, 
            25, 
            4, 
            ArmSetPoints.AMP.angleDeg,
            PhotonPositions.AMP_BLUE,
            PhotonPositions.AMP_RED
        );
    }

    /**
     * Score in speaker with vision using stateful intake/shooter
     * 
     * @param driver joystick
     * @param bluePosition PhotonPosition for the blue alliance
     * @param redPosition PhotonPosition for the red alliance
     */
    public static Command statefulSpeakerScoreWithVision(CommandXboxController driver, ArmSetPoints armAngle, PhotonPositions bluePosition, PhotonPositions redPosition) {
        return CombinedCommands.visionScoreTeleopStateful(
            driver, 
            25, 
            4, 
            armAngle.angleDeg,
            bluePosition,
            redPosition
        );
    }
}