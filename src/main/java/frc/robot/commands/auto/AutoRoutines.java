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
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.subsystems.IntakeStatesVoltage;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterStateVoltage;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoRoutines extends SubsystemBase {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("4 note");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("Diagonal45Degrees");
    PathPlannerPath BenPath = PathPlannerPath.fromPathFile("ben ");
    PathPlannerPath SpeakerPath = PathPlannerPath.fromPathFile("Speaker Path");
    PathPlannerAuto SequentialAutoTest = new PathPlannerAuto("Sequential Auto Test");
    PathPlannerAuto ParallelAutoTest = new PathPlannerAuto("Parallel Auto Test");
    PathPlannerAuto SequentialAndParallelAutoTest = new PathPlannerAuto("Sequential and Parallel Auto Test");
    //PathPlannerAuto tune = new PathPlannerAuto("tuningAuto");
    PathPlannerAuto testIntakeMotor = new PathPlannerAuto("MakeIntakeMotorSpin Auto Test");
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

        NamedCommands.registerCommand("MakeIntakeMotorSpin", new SequentialCommandGroup(
            new MakeIntakeMotorSpin(3.0,2), // Move arm to intake setpoint
            new WaitCommand(1)
        ));

        NamedCommands.registerCommand("SetModeIntake", 
            Commands.runOnce(() -> IntakeSubsystem.getInstance().setMode(IntakeStatesVoltage.Modes.INTAKE)));

        NamedCommands.registerCommand("ShootNoteStateful", 
        Commands.sequence(
            Shooter.getInstance().prepare4Speaker(),
            IntakeSubsystem.getInstance().shootNote(),
            Commands.runOnce(()->Shooter.getInstance().setMode(ShooterStateVoltage.Modes.STOP_SHOOTER))          
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
            case 3:
                return SequentialAutoTest;
            case 4:
                return ParallelAutoTest;
            case 5:
                return SequentialAndParallelAutoTest;
            case 6:
                return testIntakeMotor;
            case 7:
                return Commands.sequence(
                    SwerveSubsystem.getInstance().setOdometryCommand(BenPath.getPreviewStartingHolonomicPose()),
                    AutoBuilder.followPath(BenPath)
                );
            case 8:
                return Commands.sequence(
                    SwerveSubsystem.getInstance().setOdometryCommand(SpeakerPath.getPreviewStartingHolonomicPose()),
                    AutoBuilder.followPath(SpeakerPath)
                );
        }
    }
}