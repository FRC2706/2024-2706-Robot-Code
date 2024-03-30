package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.lib2706.SelectByAllianceCommand;
import frc.robot.Config;
import frc.robot.Config.ArmConfig;
import frc.robot.Config.PhotonConfig;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.commands.CombinedCommands;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.PhotonMoveToTarget;
import frc.robot.commands.SetArm;
import frc.robot.commands.Shooter_PID_Tuner;
import frc.robot.subsystems.IntakeStateMachine.IntakeModes;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeStateMachine.IntakeModes;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterStateMachine.ShooterModes;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoRoutines extends SubsystemBase {
    
    // PathPlannerPath speakerPath = PathPlannerPath.fromPathFile("Speaker Path");
   
    PathPlannerAuto fourNoteAuto,
                    // twoNoteAuto,
                    // threeNoteAuto,
                    twoNoteLeftAuto,
                    twoNoteCenter,
                    threeNoteCenterSourceSideNote,
                    threeNoteCenterAmpSideNote,
                    oneNoteSourceSide,
                    twoNoteSourceSide;
    

    public AutoRoutines() {
        registerCommandsToPathplanner();

        // twoNoteAuto = new PathPlannerAuto("twoNoteSpeaker");
        // threeNoteAuto = new PathPlannerAuto("threeNoteSpeaker");
        fourNoteAuto = new PathPlannerAuto("4NoteCenterSimple");
        twoNoteLeftAuto = new PathPlannerAuto("2NoteLeft");

        twoNoteCenter = new PathPlannerAuto("2NoteCenter");
        threeNoteCenterSourceSideNote = new PathPlannerAuto("3NoteCenterSourceSideNote");
        threeNoteCenterAmpSideNote = new PathPlannerAuto("3NoteCenterAmpSideNote");
        oneNoteSourceSide = new PathPlannerAuto("1NoteSourceSide");
        twoNoteSourceSide = new PathPlannerAuto("2NoteSourceSideFar");
    }

    public void registerCommandsToPathplanner() {
        NamedCommands.registerCommand("MakeIntakeMotorSpin", new SequentialCommandGroup(
            new MakeIntakeMotorSpin(3.0,2), // Move arm to intake setpoint
            new WaitCommand(1)
        ));

        NamedCommands.registerCommand("IntakeControlFalse", new IntakeControl(false));

        NamedCommands.registerCommand("SetModeIntake", 
            Commands.runOnce(() -> IntakeSubsystem.getInstance().setMode(IntakeModes.INTAKE)));

        NamedCommands.registerCommand("ShootNoteStateful", 
        Commands.sequence(
            Commands.runOnce(() -> {System.out.println("STEP1");}),
            ShooterSubsystem.getInstance().speedUpForSpeakerCommand(),
            Commands.runOnce(() -> {System.out.println("STEP2");}),
            IntakeSubsystem.getInstance().shootNoteCommand(),
            Commands.runOnce(() -> {System.out.println("STEP3");}),
            Commands.runOnce(()->ShooterSubsystem.getInstance().setMode(ShooterModes.STOP_SHOOTER))          
        ));

        NamedCommands.registerCommand("simpleShooter", CombinedCommands.simpleShootNoteSpeaker(0.4));
        
        NamedCommands.registerCommand("MakeShooterSpin", new Shooter_PID_Tuner(() -> Config.ShooterConstants.subwooferRPM));

        // Commands.deadline(
        //       Commands.sequence(
        //         new IntakeControl(false).withTimeout(0.3), 
        //         new WaitCommand(0.5),
        //         new IntakeControl(true).withTimeout(2)),
        //       new Shooter_Voltage(()->5)
        //     ));

        // NamedCommands.registerCommand("turnOffIntake", (
        //     Commands.runOnce(()-> IntakeSubsystem.getInstance().setMode(IntakeStatesVoltage.Modes.STOP))));
        
        // NamedCommands.registerCommand("turnOnIntake", (
        //         Commands.runOnce(()-> IntakeSubsystem.getInstance().setMode(IntakeStatesVoltage.Modes.INTAKE))));

        NamedCommands.registerCommand("simpleIntake", (
                new MakeIntakeMotorSpin(7.0,0)));

        // NamedCommands.registerCommand("alignToSpeaker", (
        //     PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.FAR_SPEAKER_RED)));



        NamedCommands.registerCommand("ResetToSpeakerTag",
            new SelectByAllianceCommand(
                PhotonSubsystem.getInstance().getResetCommand(7), // Blue alliance
                PhotonSubsystem.getInstance().getResetCommand(4) // Red alliance
            )
        );

        NamedCommands.registerCommand("MoveToCenterSpeaker",
            new SelectByAllianceCommand(
                new PhotonMoveToTarget(PhotonPositions.MIDDLE_SPEAKER_BLUE.destination, false, false), 
                new PhotonMoveToTarget(PhotonPositions.MIDDLE_SPEAKER_RED.destination, false, false)
            )
        );

        NamedCommands.registerCommand("ArmStartConfig", new SetArm(() -> 90).until(() -> ArmSubsystem.getInstance().getPosition() > Math.toRadians(82)));
        NamedCommands.registerCommand("ArmPickup", new SetArm(() -> Config.ArmSetPoints.INTAKE.angleDeg));
        NamedCommands.registerCommand("ArmKitbotShot", new SetArm(() -> Config.ArmSetPoints.SPEAKER_KICKBOT_SHOT.angleDeg));

        // Working but behaves weirdly with pathplanner
        NamedCommands.registerCommand("VisionScoreSourceSideClose",
            autoSpeakerScore(
                8,
                2,
                3750,
                3720,
                31,
                PhotonPositions.RIGHT_SPEAKER_BLUE,
                PhotonPositions.LEFT_SPEAKER_RED
            )
        );

        NamedCommands.registerCommand("SetVisionSideSpeakerTag", 
            new SelectByAllianceCommand(
                PhotonSubsystem.getInstance().getResetCommand(7),
                PhotonSubsystem.getInstance().getResetCommand(3)));
    }

    public Command getAutonomousCommand(int selectAuto) {
        switch (selectAuto) {
            case 0:
            default: 
                return null;
            case 1:
                return twoNoteLeftAuto;
            case 2:
                return fourNoteAuto;
            case 3:
                return oneNoteSourceSide;
            case 4:
                return twoNoteSourceSide;
            case 5:
                return threeNoteCenterSourceSideNote;
            case 6:
            case 7:
                var alliance = DriverStation.getAlliance();

                // Default to blue alliance
                if (alliance.isEmpty()) {
                DriverStation.reportWarning("Unable to detect alliance color.", false);
                    return new InstantCommand();
                }
                return Commands.sequence(
                    SwerveSubsystem.getInstance().setOdometryCommand(new Pose2d(0, 0, SwerveSubsystem.rotateForAlliance(Rotation2d.fromDegrees(0)))),
                    SwerveSubsystem.getInstance().getDriveToPoseCommand(new Pose2d((alliance.get() == DriverStation.Alliance.Blue)? 2.5 : -2.5, 0, SwerveSubsystem.rotateForAlliance(Rotation2d.fromDegrees(0))))
                );
        }
    }

    /**
     * Drive to the given photon position (blue or red) and score a note.
     * 
     * This command group will not reset vision to the correct tag. That must be done ahead of this command running.
     * 
     * @param preparingTimeoutSeconds Time in seconds before the preparing commands are canceled.
     * @param scoringTimeoutSeconds Time in seconds for the scoring commands. Scoring commands do not end automatically and rely on this.
     * @param shooterSpeed Setpoint for the shooter in RPM.
     * @param shooterTriggerSpeed Shooter speed in RPM to consider the shooter ready to shoot.
     * @param armAngleDeg Angle to put the arm at in degrees.
     * @param bluePosition PhotonPosition to drive the chassis to when on the blue alliance.
     * @param redPosition PhotonPosition to drive the chassis to when on the red alliance.
     * @return
     */
    public static Command autoSpeakerScore(
        double preparingTimeoutSeconds,
        double scoringTimeoutSeconds, 
        double shooterSpeed,
        double shooterTriggerSpeed,
        double armAngleDeg, 
        PhotonPositions bluePosition, 
        PhotonPositions redPosition
    ) {
        // Intake and shooter sequence
        // Spin the intake forwards to center the note, when the chassis is not rotating for a bit, log the centered note in the intake rollers
        Debouncer notRotatingDebouncer = new Debouncer(0.3);
        Command intakeShooterSequence = Commands.sequence(
            new MakeIntakeMotorSpin(8.0, 0).until(() -> notRotatingDebouncer.calculate(
                // SwerveSubsystem.getInstance().isAtRotationTarget(30, 5))),
                Math.abs(SwerveSubsystem.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond) < Math.toRadians(3))),
            Commands.parallel(
                new IntakeControl(false), // Reverse note until not touching shooter
                new WaitCommand(0.1).andThen(new Shooter_PID_Tuner(() -> shooterSpeed))
            )
        );

        // Prepare the robot to score
        Debouncer shooterDebounce = new Debouncer(0.2);
        Command driveToPositionAndPrepare = Commands.deadline(
            Commands.parallel(
                new WaitUntilCommand(() -> shooterDebounce.calculate(ShooterSubsystem.getInstance().getVelocityRPM() > shooterTriggerSpeed)),
                new WaitUntilCommand(() -> Math.abs(Math.toDegrees(ArmSubsystem.getInstance().getPosition()) - armAngleDeg) < 0.5),
                new WaitUntilCommand(() -> SwerveSubsystem.getInstance().isAtPose(PhotonConfig.POS_TOLERANCE, PhotonConfig.ANGLE_TOLERANCE) 
                                        && !SwerveSubsystem.getInstance().isChassisMoving(PhotonConfig.VEL_TOLERANCE))
            ),
            intakeShooterSequence,
            new SelectByAllianceCommand(
                new PhotonMoveToTarget(bluePosition.destination, bluePosition.direction, false, true),
                new PhotonMoveToTarget(redPosition.destination, redPosition.direction, false, true)),
            new SetArm(()->armAngleDeg)
        );

        // Score the note
        Command scoreNote = Commands.parallel(
            Commands.runOnce(() -> SwerveSubsystem.getInstance().stopMotors()),
            new Shooter_PID_Tuner(() -> shooterSpeed), // Continue to hold shooter voltage
            new SetArm(()->armAngleDeg), // Continue to hold arm in the correct position
            new MakeIntakeMotorSpin(9.0, 0)
        ).withTimeout(scoringTimeoutSeconds);

        // Sequence preparing then scoring
        return Commands.sequence( 
            CombinedCommands.forcefulTimeoutCommand(
                preparingTimeoutSeconds,
                driveToPositionAndPrepare
            ),
            scoreNote
        );
    }
}
