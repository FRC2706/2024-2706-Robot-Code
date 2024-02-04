package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoRoutines extends SubsystemBase {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("2mStraightLines");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("Diagonal45Degrees");

    public AutoRoutines() {
        
    }

    public static Command visionScoreSpeakerCommand(PhotonPositions scorePos) {
        return new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                PhotonSubsystem.getInstance().getAprilTagCommand(scorePos),
                new WaitUntilCommand(() -> {return true;}), // Wait for arm to be in position
                new WaitUntilCommand(() -> {return true;}), // Wait for shooter to be spun up,
                new WaitCommand(0.3) // Move intake for shoot command
            ),
            new WaitCommand(2), // Move arm to speaker center close setpoint
            new WaitCommand(3) // Spin shooter up to speed 
        );
    }

    public static void registerCommandsToPathplanner() {
        // Vision Commands
        NamedCommands.registerCommand("VisionScoreSpeakerCenter", visionScoreSpeakerCommand(PhotonPositions.MIDDLE_SPEAKER_BLUE));

        // Intake and Arm Commands
        NamedCommands.registerCommand("IntakeAndArm", new ParallelCommandGroup(
            new WaitCommand(1), // Move arm to intake setpoint
            new WaitCommand(1) // Intake game piece
        ));
    }

    public Command getAutonomousCommand(int selectAuto) {
        switch (selectAuto) {
            case 0:
            default: 
                return null;
            case 1: 
                if (path1 == null) {
                    System.out.println("path1 path is null");
                    return null;
                }
                return Commands.sequence(
                    SwerveSubsystem.getInstance().setOdometryCommand(path1.getPreviewStartingHolonomicPose()),
                    AutoBuilder.followPath(path1)
                );
            case 2:
                return new PathPlannerAuto("testAuto");
        }
    }
}