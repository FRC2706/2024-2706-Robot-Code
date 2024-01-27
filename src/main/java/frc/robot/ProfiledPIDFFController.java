package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmConfig;

public class ProfiledPIDFFController {
    private double lastVel = 0; 
    private double lastTime = Timer.getFPGATimestamp();

    private final TrapezoidProfile.Constraints m_constraints = 
    new TrapezoidProfile.Constraints(ArmConfig.MAX_VEL, ArmConfig.MAX_ACCEL);
    private final ProfiledPIDController m_ProfiledPIDController = 
    new ProfiledPIDController(1.6,0.002,40, m_constraints, 0.02);

    private final SimpleMotorFeedforward m_FFController = new SimpleMotorFeedforward(0,0);

    public ProfiledPIDFFController ()
    {

    }

    public void setFinalGoal( TrapezoidProfile.State finalGoal) 
    {
        m_ProfiledPIDController.setGoal(finalGoal);
    }

    public double getProfiledPIDVoltage (double measuredPos, double finalPos)
    {
        return m_ProfiledPIDController.calculate(measuredPos, finalPos);
    }

    public double getProfiledPIDFFVoltage(double measuredPos, double finalPos)
    {
        double pidVal = getProfiledPIDVoltage(measuredPos, finalPos);

        double currVel = m_ProfiledPIDController.getSetpoint().velocity;
        double currAccel = (m_ProfiledPIDController.getSetpoint().velocity - lastVel)/(Timer.getFPGATimestamp() - lastTime);
        double ffVal = m_FFController.calculate(currVel, currAccel);

        lastVel = currVel;
        lastTime = Timer.getFPGATimestamp();

        return ffVal;
    }

    public double getNextProfiledPIDPos(double measuredPos, double finalPos) {
        double pidVal = getProfiledPIDVoltage(measuredPos, finalPos);

        double currPos = m_ProfiledPIDController.getSetpoint().position;
        return currPos;
    }
}

