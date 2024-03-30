package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.lib2706.SelectByAllianceCommand;
import frc.robot.Config;
import frc.robot.Config.ArmSetPoints;
import frc.robot.Config.PhotonConfig;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.commands.BlingCommand.BlingColour;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlingSubsystem;
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
        return(simpleShootNoteSpeaker(intakeTimeout, () -> Config.ShooterConstants.subwooferRPM, 100));
    }

    /**
     * Spin up the shooter while doing the following,
     * backing up note, waiting a bit, then feeding the note.
     */
    public static Command simpleShootNoteSpeaker(double intakeTimeout, DoubleSupplier RPM, double clearance) {
        return Commands.deadline(
            Commands.sequence(
                new IntakeControl(false).withTimeout(0.15), 
                new WaitUntilCommand(() -> ShooterSubsystem.getInstance().getVelocityRPM() > RPM.getAsDouble()),
                new IntakeControl(true).withTimeout(intakeTimeout)),
            new Shooter_PID_Tuner(()->(RPM.getAsDouble() + clearance))
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
            new SetArm(()->ArmSetPoints.INTAKE.angleDeg), // Continue to hold arm in the correct position
            PhotonSubsystem.getInstance().saveImagesIntakeCameraCommand()
        );
    }

    /**
     * Centers the note then spins up the shooter.
     * 
     * The belts naturally center the note while the intake is spinning as long as the robot is not rotating very fast.
     * This command group will spin the intake rollers until the chassis has not rotated for 0.3 seconds then spinup the shooter.
     * 
     * @param shooterSpeed The speed in RPM to set the shooter to.
     */
    public static Command centerNoteThenSpinUpShooer(double shooterSpeed) {
        // Intake and shooter sequence
        // Spin the intake forwards to center the note, when the chassis is not rotating for a bit, lodge the centered note in the intake rollers
        Debouncer notRotatingDebouncer = new Debouncer(0.5);
        return Commands.sequence(
            Commands.runOnce(() -> notRotatingDebouncer.calculate(false)),
            new MakeIntakeMotorSpin(8.0, 0).until(() -> notRotatingDebouncer.calculate(
                Math.abs(SwerveSubsystem.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond) < Math.toRadians(3))),
            Commands.parallel(
                new IntakeControl(false), // Reverse note until not touching shooter
                new WaitCommand(0.2).andThen(new Shooter_PID_Tuner(() -> shooterSpeed))
            )
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
            BooleanSupplier keepArmLoweredUntil,
            PhotonPositions bluePosition, 
            PhotonPositions redPosition) {

        // Use a timer to not rumble if the it's only been 0.5 seconds
        Timer timer = new Timer();

        // Bling Commands
        Command bling = new ProxyCommand(new BlingCommand(BlingColour.BLUESTROBE));
        Command idleBling = new ProxyCommand(Commands.idle(BlingSubsystem.getINSTANCE()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("ProxiedIdleBling"));
        Command turnOffBling = new ProxyCommand(new BlingCommand(BlingColour.DISABLED).withName("TurnOffBling"));

        // Wait for vision data to be available
        Command waitForVisionData = Commands.deadline(new ProxyCommand(new SelectByAllianceCommand(
            PhotonSubsystem.getInstance().getWaitForDataCommand(bluePosition.id), 
            PhotonSubsystem.getInstance().getWaitForDataCommand(redPosition.id)).withName("ProxiedWaitForVisionData")),
            new ProxyCommand(Commands.parallel(
                new IntakeControl(false), // Reverse note until not touching shooter
                new WaitCommand(0.2).andThen(new Shooter_PID_Tuner(() -> shooterSpeed))
            ).withName("ProxiedReverseNoteAndSpinupShooter"))
        );
            
        // Wait for all subsytems to get ready
        Command waitForAllSubsytems = Commands.parallel(
            new WaitUntilCommand(() -> ShooterSubsystem.getInstance().getVelocityRPM() > shooterTriggerSpeed),
            new WaitUntilCommand(() -> Math.abs(Math.toDegrees(ArmSubsystem.getInstance().getPosition()) - armAngleDeg) < 1),
            new WaitUntilCommand(() -> SwerveSubsystem.getInstance().isAtPose(PhotonConfig.POS_TOLERANCE, PhotonConfig.ANGLE_TOLERANCE) 
                                    && !SwerveSubsystem.getInstance().isChassisMoving(PhotonConfig.VEL_TOLERANCE))
        );

        // Control all subsystems commands
        Command controlAllSubsystems = Commands.parallel(
            new WaitUntilCommand(keepArmLoweredUntil).andThen(new SetArm(()->armAngleDeg)),
            // new ProxyCommand(centerNoteThenSpinUpShooer(shooterSpeed).withName("ProxiedCenterNoteThenSpinUpShooter")),
            new ProxyCommand(Commands.parallel(
                new IntakeControl(false), // Reverse note until not touching shooter
                new WaitCommand(0.1).andThen(new Shooter_PID_Tuner(() -> shooterSpeed))
            ).withName("ProxiedReverseNoteAndSpinupShooter")),
            new SelectByAllianceCommand(
                    PhotonSubsystem.getInstance().getAprilTagCommand(bluePosition, driverJoystick, true), 
                    PhotonSubsystem.getInstance().getAprilTagCommand(redPosition, driverJoystick, true)),
            Commands.sequence(bling, new WaitCommand(0.02), idleBling),
            Commands.runOnce(() -> timer.restart())
        );

        // Score the note
        Command scoreNote = new ProxyCommand(
            Commands.parallel(
                new MakeIntakeMotorSpin(9.0, 0),
                new Shooter_PID_Tuner(() -> shooterSpeed)
            ).withName("ProxiedShooterAndFeedIntake")
        ).withTimeout(scoringTimeoutSeconds);

        // Rumble command
        Command rumble = new RumbleJoystick(driverJoystick, RumbleType.kBothRumble, 0.7, 0.3, false);

        // Sequence preparing then scoring
        return Commands.sequence( 
            waitForVisionData,
            Commands.deadline(
                Commands.sequence(
                    forcefulTimeoutCommand(
                        preparingTimeoutSeconds,
                        waitForAllSubsytems
                    ),
                    scoreNote),
                controlAllSubsystems
            )
        ).finallyDo(() -> {
            idleBling.cancel(); // Cancel idle bling as a safety factor
            Commands.sequence(new WaitCommand(0.02), new ScheduleCommand(turnOffBling)).withName("DelayTurnOffBling").schedule();
            if (timer.hasElapsed(1))
                rumble.schedule(); // Rumble the joystick to notify the driver
        }).withName("VisionScoreTeleopSimple");
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
                    PhotonSubsystem.getInstance().getAprilTagCommand(bluePosition, driverJoystick, false), 
                    PhotonSubsystem.getInstance().getAprilTagCommand(redPosition, driverJoystick, false)),
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
        Command rumble = new RumbleJoystick(driverJoystick, RumbleType.kBothRumble, 0.7, 0.3, false);

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
            () -> true,
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
    public static Command centerSpeakerVisionShot(CommandXboxController driver, PhotonPositions bluePosition, PhotonPositions redPosition) {
        BooleanSupplier keepArmLoweredUntil = () -> {
          return PhotonSubsystem.getInstance().getTargetPos().getY() - SwerveSubsystem.getInstance().getPose().getY() < 0.5;
        };
        
        double armAngle = 41;
        double shooterSpeed = 4000;
        double shooterTriggerSpeed = 3960;

        return CombinedCommands.visionScoreTeleopSimple(
            driver, 
            12, 
            1,
            shooterSpeed, shooterTriggerSpeed,
            armAngle,
            keepArmLoweredUntil,
            bluePosition,
            redPosition
        );
    }

    /**
     * Score in speaker with vision using simple intake/shooter.
     * 
     * @param driver joystick
     * @param bluePosition PhotonPosition for the blue alliance
     * @param redPosition PhotonPosition for the red alliance
     */
    public static Command podiumSourceSideSpeakerVisionShot(CommandXboxController driver, PhotonPositions bluePosition, PhotonPositions redPosition) {
        // BooleanSupplier keepArmLoweredUntil = () -> {
        //   return PhotonSubsystem.getInstance().getTargetPos().getY() - SwerveSubsystem.getInstance().getPose().getY() > 0.5;
        // };
        
        BooleanSupplier keepArmLoweredUntil = () -> {return true;};

        double armAngle = 39;
        double shooterSpeed = 4000;
        double shooterTriggerSpeed = 3960;

        return CombinedCommands.visionScoreTeleopSimple(
            driver, 
            12, 
            1,
            shooterSpeed, shooterTriggerSpeed,
            armAngle,
            keepArmLoweredUntil,
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
     /*Bling command to indicate that a note is loaded in intake*/
    public static Command strobeToSolidBlingCommand() {
        return
            Commands.sequence(
                new BlingCommand(BlingColour.PURPLESTROBE),
                new WaitCommand(2),
                new BlingCommand(BlingColour.PURPLE))
        ;
    }
    
}