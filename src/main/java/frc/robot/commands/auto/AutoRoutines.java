package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoRoutines extends SubsystemBase {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("2mStraightLines");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("Diagonal45Degrees");

    public AutoRoutines() {
        
    }

    public Command getAutonomousCommand(int selectAuto) {
        System.out.println (selectAuto);
        switch (selectAuto) {
            case 0: 
                return null;
            case 1: 
                return AutoBuilder.followPathWithEvents(path1);
            case 2:
                return AutoBuilder.followPathWithEvents(path2);
        }
    }
}