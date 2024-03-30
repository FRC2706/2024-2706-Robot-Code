// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SubwooferShot extends Command {
    private final double SHORT_PERIOD = 0.3;

    private double m_armAngleRad;
    private double m_shooterRPM;
    private double m_shooterTriggerRPM;

    private Timer m_timer = new Timer();
    private BooleanSupplier m_isButtonPressed;
    private boolean dontEndUntilButtonPressed = false;
    private boolean hasPassedShortPeriod = false;
    private Command reverseIntakeCommand;
    private Command feedNoteCommand;

    /** Creates a new SubwooferShot. */
    public SubwooferShot(BooleanSupplier isButtonPressed, double armAngleDeg, double shooterRPM, double shooterTriggerRPM) {
        m_isButtonPressed = isButtonPressed;

        m_armAngleRad = Math.toRadians(armAngleDeg);
        m_shooterRPM = shooterRPM;
        m_shooterTriggerRPM = shooterTriggerRPM;

        reverseIntakeCommand = new IntakeControl(false);
        feedNoteCommand = new MakeIntakeMotorSpin(9.0, 0);

        addRequirements(ArmSubsystem.getInstance(), ShooterSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.restart();
        dontEndUntilButtonPressed = false;
        hasPassedShortPeriod = false;

        reverseIntakeCommand.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ArmSubsystem.getInstance().setJointAngle(m_armAngleRad);
        ShooterSubsystem.getInstance().setRPM(m_shooterRPM);        

        if (!hasPassedShortPeriod && m_timer.hasElapsed(SHORT_PERIOD)) {
            hasPassedShortPeriod = true;

            if (!m_isButtonPressed.getAsBoolean()) {
                dontEndUntilButtonPressed = true;
            }
        }

        if (dontEndUntilButtonPressed && m_isButtonPressed.getAsBoolean()) {
            dontEndUntilButtonPressed = false;
        }

        if (m_isButtonPressed.getAsBoolean() && ShooterSubsystem.getInstance().getVelocityRPM() > m_shooterTriggerRPM) {
            feedNoteCommand.schedule();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        reverseIntakeCommand.cancel();
        feedNoteCommand.cancel();
        ShooterSubsystem.getInstance().setRPM(0);
        IntakeSubsystem.getInstance().setVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (dontEndUntilButtonPressed) {
            return false;
        }

        return hasPassedShortPeriod && !m_isButtonPressed.getAsBoolean();
    }
}
