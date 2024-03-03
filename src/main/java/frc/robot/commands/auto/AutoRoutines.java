package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.lib2706.SelectByAllianceCommand;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.commands.CombinedCommands;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.PhotonMoveToTarget;
import frc.robot.commands.Shooter_Voltage;
import frc.robot.subsystems.IntakeStatesMachine.IntakeModes;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterStateMachine.ShooterModes;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoRoutines extends SubsystemBase {
    
    // PathPlannerPath speakerPath = PathPlannerPath.fromPathFile("Speaker Path");
    PathPlannerAuto twoNoteAuto = new PathPlannerAuto("twoNoteSpeaker");
    PathPlannerAuto threeNoteAuto = new PathPlannerAuto("threeNoteSpeaker");
    PathPlannerAuto fourNoteAuto = new PathPlannerAuto("4NoteCenter");
    PathPlannerAuto twoNoteLeftAuto = new PathPlannerAuto("2NoteLeft");

    public AutoRoutines() {
        registerCommandsToPathplanner();
    }

    public void registerCommandsToPathplanner() {
        // Intake and Arm Commands
        NamedCommands.registerCommand("IntakeAndArm", new ParallelCommandGroup(
            new WaitCommand(1), // Move arm to intake setpoint
            new WaitCommand(1) // Intake game piece
        ));

        NamedCommands.registerCommand("OutakeRing", new ParallelCommandGroup(
            new WaitCommand(1), // Move arm to speaker 
            new WaitCommand(1) // Outake game piece
        ));

        NamedCommands.registerCommand("StartingZoneAmp", new ParallelCommandGroup(
            new WaitCommand(1), // Exit starting zone
            new WaitCommand(1), // Intake note
            new WaitCommand(1) // Score in amp
        ));

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

        NamedCommands.registerCommand("simpleShooter", CombinedCommands.simpleShootNoteSpeaker());
        
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
                new PhotonMoveToTarget(PhotonPositions.MIDDLE_SPEAKER_BLUE.destination, false), 
                new PhotonMoveToTarget(PhotonPositions.MIDDLE_SPEAKER_RED.destination, false)
            )
        );
    }

    public Command getAutonomousCommand(int selectAuto) {
        switch (selectAuto) {
            case 0:
            default: 
                return null;
            case 1:
                return null;
                // return Commands.sequence(
                //     SwerveSubsystem.getInstance().setOdometryCommand(speakerPath.getPreviewStartingHolonomicPose()),
                //     AutoBuilder.followPath(speakerPath)
                // );
            case 2:
                return twoNoteAuto;
            case 3:
                return threeNoteAuto;
            case 4:
                return fourNoteAuto;
            case 5:
                return twoNoteLeftAuto;
        }
    }
}
