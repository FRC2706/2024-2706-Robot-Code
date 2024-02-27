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
import frc.robot.commands.IntakeControl;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.Shooter_tuner;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.PhotonMoveToTarget;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.lib.lib2706.SelectByAllianceCommand;
import frc.robot.Config.PhotonConfig.PhotonPositions;

public class AutoRoutines extends SubsystemBase {
    PathPlannerPath SpeakerPath = PathPlannerPath.fromPathFile("Speaker Path");
    PathPlannerAuto twoNoteAuto = new PathPlannerAuto("twoNoteSpeaker");
    PathPlannerAuto threeNoteAuto = new PathPlannerAuto("threeNoteSpeaker");
    private static IntakeSubsystem intake = IntakeSubsystem.getInstance();

    public AutoRoutines() {
        
    }

    // private static AutoRoutines instance;
    // public static AutoRoutines getInstance(){
    //     if(instance == null){
    //         instance = new AutoRoutines();
    //     }
    //     return instance;
    // }

    public static void registerCommandsToPathplanner() {
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

        NamedCommands.registerCommand("IntakeAndArm", new ParallelCommandGroup(
            new WaitCommand(1), // Move arm to intake setpoint
            new WaitCommand(1) // Intake game piece
        ));
        NamedCommands.registerCommand("MakeIntakeMotorSpin", new SequentialCommandGroup(
            new MakeIntakeMotorSpin(3.0,2), // Move arm to intake setpoint
            new WaitCommand(1)
        ));

        
        NamedCommands.registerCommand("shooter", new SequentialCommandGroup(
            Commands.deadline(
                Commands.sequence(
                    Commands.waitSeconds(2), 
                    IntakeSubsystem.getInstance().shootNote()
                ),
                new Shooter_tuner(()->5)
            )
        ));

        NamedCommands.registerCommand("simpleShooter", Commands.deadline(
              Commands.sequence(
                new IntakeControl(false).withTimeout(0.3), 
                new WaitCommand(0.5),
                new IntakeControl(true).withTimeout(2)),
              new Shooter_tuner(()->5)
            ));

        // NamedCommands.registerCommand("turnOffIntake", (
        //     Commands.runOnce(()-> IntakeSubsystem.getInstance().setMode(STOP))));
        
        // NamedCommands.registerCommand("turnOnIntake", (
            
        //         Commands.runOnce(()-> IntakeSubsystem.getInstance().setMode(INTAKE))));

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
                return Commands.sequence(
                    SwerveSubsystem.getInstance().setOdometryCommand(SpeakerPath.getPreviewStartingHolonomicPose()),
                    AutoBuilder.followPath(SpeakerPath)
                );
            case 2:
                return twoNoteAuto;
            case 3:
                return threeNoteAuto;
        }
    }
}
