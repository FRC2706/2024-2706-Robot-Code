package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BlingCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoRoutines extends SubsystemBase {
    PathPlannerAuto four_note = new PathPlannerAuto("4note");
    PathPlannerAuto SequentialAutoTest = new PathPlannerAuto("Sequential Auto Test");
    PathPlannerAuto ParallelAutoTest = new PathPlannerAuto("Parallel Auto Test");
    PathPlannerAuto SequentialAndParallelAutoTest = new PathPlannerAuto("Sequential and Parallel Auto Test");
    PathPlannerAuto tuneX = new PathPlannerAuto("tuneAutoX");
    PathPlannerAuto tuneY = new PathPlannerAuto("tuneAutoY");
    public AutoRoutines() {
        
    }


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

        NamedCommands.registerCommand("BlingYellow", new BlingCommand(BlingCommand.BlingColour.YELLOW));
        NamedCommands.registerCommand("BlingRed", new BlingCommand(BlingCommand.BlingColour.RED));
    }

    

    public Command getAutonomousCommand(int selectAuto) {
        switch (selectAuto) {
            case 0:
            default: 
                return new InstantCommand();
            case 1:
                return new SequentialCommandGroup(
                    SwerveSubsystem.getInstance().setOdometryCommand(new Pose2d(1,1, new Rotation2d(0))),
                    SwerveSubsystem.getInstance().getDriveToPoseCommand(new Pose2d(3, 1, Rotation2d.fromDegrees(0)))
                );
            case 2:
                return tuneX;
            case 3:
                return tuneY;
            case 4:
                return SequentialAutoTest;
            case 5:
                return ParallelAutoTest;
            case 6:
                return SequentialAndParallelAutoTest;
            case 7:
                return four_note;
        }
    }
}