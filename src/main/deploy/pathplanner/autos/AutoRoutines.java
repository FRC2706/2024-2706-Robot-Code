import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoRoutines extends SubsystemBase {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("2mStraightLines");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("Diagonal45Degrees");
    private final SwerveSubsystem s_Swerve = SwerveSubsystem.getInstance();
    

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
            case 3:
                return new SequentialCommandGroup(
                    SwerveSubsystem.getInstance().resetOdometry(new Pose2d(1,1, new Rotation2d(0))),
                    SwerveSubsystem.getInstance().getDriveToPoseCommand(new Pose2d(3, 1, 0))
                );
        }
    }

}
