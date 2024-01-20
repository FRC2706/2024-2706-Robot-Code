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
