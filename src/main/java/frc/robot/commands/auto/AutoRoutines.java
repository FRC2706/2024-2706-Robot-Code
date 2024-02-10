package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoRoutines extends SubsystemBase {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("2mStraightLines");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("Diagonal45Degrees");

    public AutoRoutines() {
        
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