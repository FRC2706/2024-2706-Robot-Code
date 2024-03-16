package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

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
import frc.lib.lib2706.SelectByAllianceCommand;
import frc.robot.Config;
import frc.robot.Config.ArmConfig;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.commands.CombinedCommands;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.PhotonMoveToTarget;
import frc.robot.commands.SetArm;
import frc.robot.subsystems.IntakeStatesMachine.IntakeModes;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeStatesMachine.IntakeModes;
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
                return new PathPlannerAuto("tuneAutoX");
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
}
